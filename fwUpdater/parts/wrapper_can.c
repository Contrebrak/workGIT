/**
 * This file defines all jobs required to update the microcontrollers via
 * the CAN interface
 */

#include "include/fw_wrapper_can.h"

#ifdef __GNUC__
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#else
#define likely(x)       (x)
#define unlikely(x)     (x)
#endif

/** Constants and defines **/

#define APP_BASE_ADDR         0x10000
#define APP_FLASH_SIZE        0x43FFC // In bytes
#define APP_LAST_ADDR         (APP_BASE_ADDR + APP_FLASH_SIZE)
#define END_OF_FILE           1
#define CAN_RESPONSE_TIMEOUT  15
#define CAN_REBOOT_TIMEOUT    20
#define CRC_HIGH_ADDR         0x5
#define CRC_LOW_ADDR          0x3FFC

#define CAN_INACK_BIT   0x1
#define CAN_INACK_LSH   0
#define CAN_INRDY_BIT   0x2
#define CAN_INRDY_LSH   1
#define CAN_OUTACK_BIT  0x1
#define CAN_OUTACK_LSH  0
#define CAN_OUTRDY_BIT  0x2
#define CAN_OUTRDY_LSH  1

#define CAN_ID_LLD_OFFSET     0x00
#define CAN_ID_APFB_OFFSET    0x08
#define CAN_ID_HIL_OFFSET     0x10

#define CAN_ID_REBOOT_IN_BSL  0x580
#define CAN_ID_VERSION        0x780
#define CAN_ID_WAKEUP_FRAME   0x7C0
#define CAN_ID_BSL_ACTION     0x550 + 8
#define CAN_ID_BSL_PROGRAM_APP 0x590 + 8
#define CAN_ID_BSL_STATUS     0x6D0 + 8
#define CAN_ID_BSL_VERSION    0x710 + 8
#define CAN_ID_BSL_WAKEUP     0x750 + 8

#define CAN_DISABLED_INCOMING 0x999
#define CAN_MAX_RETRIES 5
/* Size of CAN frame payload */
#define SIZE_OF_CAN_FRAME_DATA 6

/**
 * @brief Convert 'e_canCard' typed enumeration to a string
 */
#define mcu_to_string(type) type == e_canCardAPFB ? "APFB" : \
								  type == e_canCardHIL ? "HIL" : "LLD"

/**
 * @brief Array of existing LLD cards IDs
 */
static const unsigned int LLD_CARD_IDS[NB_LLD_NODES] = {2, 3};
/**
 * @brief Array of existing APFB cards IDs
 */
static const unsigned int APFB_CARD_IDS[NB_APFB_NODES] = {1, 4};
/**
 * @brief Array of existing HIL cards IDs
 *
 * The 'pnp' of HIL is '5' for BSL mode, but '1' for APP mode. This last case
 * is managed by the function wrapper_can_calculate_frame_id()
 */
static const unsigned int HIL_CARD_IDS[NB_HIL_NODES] = {5};


/** Enum and structs **/

/**
 * @brief State of a CAN node
 */
typedef enum {
	/**
	 * @brief Node is up (not used)
	 */
	CAN_NODE_OK,
	/**
	 * @brief Bus error (not used)
	 */
	CAN_NODE_BUS_ERROR,
	/**
	 * @brief Undetected or not responding CAN node
	 */
	CAN_NODE_UNDETECTED,
	/**
	 * @brief Detected CAN node in BSL firmware
	 */
	CAN_NODE_IN_BSL,
	/**
	 * @brief Detected CAN node in applicative firmware
	 */
	CAN_NODE_IN_APP
} e_canReturnCode;

/**
 * @struct s_canFrame
 * @brief CAN frame (embedded in a Openpowerlink PDO)
 *
 * @var s_canFrame::id
 *  CAN ID
 * @var s_canFrame::dlc
 *  Size of useful data
 * @var s_canFrame::data
 *  Buffer for the data of the frame
 */
typedef struct s_canFrame {
  unsigned int id;
  unsigned char dlc;
  unsigned char data[8];
} s_canFrame;

/**
 * @struct can_ctxt
 * @brief Struct shared by FwUpdater and its PDO handler. Used
 * to transmit between them incoming/outcoming frames
 *
 * @var can_ctxt::input
 *  Buffer and metadata for input frame exchanged between FwUpdater and its
 *  PDO handler
 * @var can_ctxt::output
 *  Buffer and metadata for output frame exchanged between FwUpdater and its
 *  PDO handler
 * @var can_ctxt::expected_id
 *  Expected CAN ID to receive. Frame receiving is disabled if this field is
 *  equal to CAN_DISABLED_INCOMING. Protected by input mutex
 * @var can_ctxt::high_addr
 *  High address for uploading. No protected by mutexes
 * @var can_ctxt::low_addr
 *  Low address for uploading. No protected by mutexes
 *
 * @struct can_ctxt::can_shared_frame
 * @brief Contains mutex, condtion and buffers to store and exchange one frame
 * between FwUpdater and the PDO handler. Unidirectional
 *
 * @var can_ctxt::can_shared_frame::mutex
 *  Pthread mutex used to wait for an event 
 * @var can_ctxt::can_shared_frame::condStackOff
 *  Pthread condition used to wait for an event
 * @var can_ctxt::can_shared_frame::frame
 *  Frame buffer
 * @var can_ctxt::can_shared_frame::frame_stored
 *  Frame currently stored or not
 */
struct can_ctxt {
	struct can_shared_frame {
		pthread_mutex_t mutex;
		pthread_cond_t condStackOff;
		s_canFrame frame;
		bool frame_stored;
	} input, output;
	unsigned int expected_id;
	unsigned char high_addr[2];
	unsigned char low_addr[2];
} can_ctxt;

/**
 * @brief Type of CRC query
 */
typedef enum {
	/**
	 * @brief Get CRC from CAN node internal calculation
	 */
	eCanCrcFromCalculation,
	/**
	 * @brief Get CRC from CAN node internal memory
	 */
	eCanCrcFromMemory
} eCanCrcFrom;

static struct wrapper_SDO_context wrapper_SDO_ctxt = {
	.error = 0x0,
	.sent = 0,
};

static struct wrapper_PDO_context wrapper_PDO_ctxt = {
	.cmd_word = -1,
	.cmd_ret = -1,
	.send_cmd_word = false,
	.get_cmd_ret = false,
	.error = false,
};

/*
 * Errors returned by microcontrollers (via status frame).
 * First bit (opened session) is not checked
 */
#define CAN_STATUS_FRAME_ERASE_FAIL (1 << 1)
#define CAN_STATUS_FRAME_PROG_FAIL (1 << 2)
#define CAN_STATUS_FRAME_NEEDED_ERASE (1 << 3)
#define CAN_STATUS_FRAME_INVALID_CMD (1 << 4)
#define CAN_STATUS_FRAME_INVALID_ARG (1 << 5)
#define CAN_STATUS_FRAME_UNKNOWN_CMD (1 << 6)
#define CAN_STATUS_FRAME_INVALID_DLC (1 << 7)

/**
 * @struct wrapper_status_frame_data_bits
 * @brief Struct containg error strings according to their associated bit
 * (from status frame)
 *
 * @var wrapper_status_frame_data_bits::bit
 *  Bit position associated to an microcontroller error
 *
 * @var wrapper_status_frame_data_bits::bit_str
 *  Error string associated to an microcontroller error
 */
struct wrapper_status_frame_data_bits {
	unsigned int bit;
	const char *bit_str;
};

static const struct wrapper_status_frame_data_bits w_status_data_debug[] = {
	{ CAN_STATUS_FRAME_ERASE_FAIL, "Erasure failure" },
	{ CAN_STATUS_FRAME_PROG_FAIL, "Programming failure" },
	{ CAN_STATUS_FRAME_NEEDED_ERASE, "Needed erasure" },
	{ CAN_STATUS_FRAME_INVALID_CMD, "Invalid command" },
	{ CAN_STATUS_FRAME_INVALID_ARG, "Invalid argument" },
	{ CAN_STATUS_FRAME_UNKNOWN_CMD, "Unknown command" },
	{ CAN_STATUS_FRAME_INVALID_DLC, "Invalid DLC" },
};

/** Static function prototypes **/

static int bsl_write_app(const unsigned int pnp, char const * const binFilename,
		char const * const crcFilename, progress_tracker_t *prog_trk);
static int send_frame_and_wait(s_canFrame snd_frame, s_canFrame *rcv_frame,
		int expected, int timeout);
static int wrapper_can_outgoing_frame_ackd(unsigned short const * const
		control_word, unsigned short const * const status_word);
static void wrapper_can_notify_new_outgoing_frame(unsigned short * const
		control_word, unsigned short const * const status_word);
static int wrapper_can_new_frame_rcvd(unsigned short const * const control_word,
		unsigned short const * const status_word);
static void wrapper_can_ack_incoming_frame(unsigned short * const
		control_word, unsigned short const * const status_word);
static unsigned int wrapper_can_calculate_frame_id(const e_canCard card,
		const unsigned int pnp, const unsigned int baseId);
static e_canReturnCode wrapper_can_detect_node(const e_canCard card,
		const unsigned int pnp);
static int get_data_from_file(FILE* fd, unsigned char
		data[SIZE_OF_CAN_FRAME_DATA]);
static unsigned int wrapper_can_read_crc(char const * const crc_file);
static int app_request_wakeup(const e_canCard card, const unsigned int pnp,
		int max_retry);
static int app_reboot_in_bsl(const e_canCard card, const unsigned int pnp);
static int bsl_request_wakeup(const unsigned int pnp, int max_retry);
static int bsl_set_high_addr(const int cardNb, const unsigned int addr);
static int bsl_write_program(const int cardNb, const unsigned char data[8]);
static int bsl_prog_validity_confirm(const unsigned int pnp);
static int bsl_open_prog_session(const unsigned int cardNb);
static int bsl_close_prog_session(const unsigned int cardNb);
static int app_get_version(const e_canCard card, const unsigned int pnp,
		char *str_version);
static int bsl_get_version(const unsigned int pnp, char *str_version);
static int bsl_get_crc(const unsigned int cardNb, const eCanCrcFrom from,
		const unsigned int crc);
static int bsl_launch_app(const e_canCard card, const unsigned int pnp);


/** Function definitions **/

/**
 * @brief Reset CAN context, used when GUI is enabled
 *
 * Reinitialize CAN context, allowing multiple consecutive updates with GUI
 */
void wrapper_can_reset_context(void)
{
	wrapper_SDO_ctxt.error = 0x0;
	wrapper_SDO_ctxt.sent = 0;

	wrapper_PDO_ctxt.cmd_word = -1;
	wrapper_PDO_ctxt.cmd_ret = -1;
	wrapper_PDO_ctxt.send_cmd_word = false;
	wrapper_PDO_ctxt.get_cmd_ret = false;
	wrapper_PDO_ctxt.error = false;
}

/**
 * @brief Analyze the status frame data and prints user-friendly string
 *
 * Each status frame data contains a bitmask meaning for its command
 * what were its result. This function return a user-friendly string
 * matching the status. It manages also the case when several status
 * bits are set, so the string will be splitted with commas
 *
 * @param[in] bitmask status frame data
 *
 * @return return a string describing the status frame data. Do free after use
 */
const char* wrapper_can_get_status_return_str(unsigned int bitmask)
{
	unsigned int i;
	char *str = malloc(sizeof(char) * 130);
	char *pos = str;

	/* String initialization */
	memset(str, '\0', sizeof(char) * 130);

	/* For each data bit */
	for (i = 0 ; i < (sizeof(w_status_data_debug) /
				sizeof(*(w_status_data_debug))) ; i++) {
		/* If checked bit is to '0' (so no error) */
		if ((bitmask & w_status_data_debug[i].bit) != w_status_data_debug[i].bit)
			continue;

		/* If this is no the only error (in order to add comma) */
		if (str != pos) {
			strncpy(pos, ", ", strlen(", "));
			pos += strlen(", ");
		}

		strncpy(pos, w_status_data_debug[i].bit_str,
				strlen(w_status_data_debug[i].bit_str));
		pos += strlen(w_status_data_debug[i].bit_str);

	}
	return str;
}

/**
 * @brief Send binary to a CAN card, validate it then launch applicative
 *
 * This function sends a binary (and its CRC) to a CAN card and validate
 * thank to calculated CRC returned by this last. Finally, it asks to
 * the CAN card to reboot to its applicative firmware
 *
 * @param[in] card type of microcontroller (APFB, HIL or LLD)
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] binFilename path to the firmware binary to flash
 * @param[in] crcFilename path to the CRC associated to the firmware binary
 * @param[in] prog_trk (GUI) structure used to follow a firmware update
 *
 * @return 0 if succeeded, -1 else
 */
static int wrapper_can_flash_app(const e_canCard card, const unsigned int pnp,
		char const * const binFilename,
		char const * const crcFilename,
		progress_tracker_t *prog_trk)
{
	int ret;

	/* Write app */
	ret = bsl_write_app(pnp, binFilename, crcFilename, prog_trk);
	if (ret != 0) {
		if (ret == -1)
			log_submit(eMessageTypeError, "Couldn't write binary to the "
					"microcontroller [name:%s pnp:%d]",
					mcu_to_string(card), pnp);

		return ret;
	}

	/* Programing validity confirmation */
	if (bsl_prog_validity_confirm(pnp) != 0) {
		log_submit(eMessageTypeError, "Couldn't confirm programming validity "
				"for the microcontroller [name:%s pnp:%d]",
				mcu_to_string(card), pnp);
		return -1;
	}

	/* Enter APP */
	log_submit(eMessageTypeNote, "Switch the microcontroller to APP mode: "
			"[name:%s pnp:%d]", mcu_to_string(card), pnp);

	/* In BSL mode, enter in APP mode */
	if (bsl_launch_app(card, pnp) != 0) {
		log_submit(eMessageTypeError, "Failed to enter in APP mode for the "
				"microcontroller [name:%s pnp:%d]",
				mcu_to_string(card), pnp);
		return -1;
	}

	log_submit(eMessageTypeNote, "Microcontroller [name:%s pnp:%d] flashed "
			"successfully !", mcu_to_string(card), pnp);

	return 0;
}

/**
 * @brief Program the applicative area memory of a microcontroller
 *
 * This functions does many things : it reads the binary file, gets its CRC,
 * opens a programming session on the microcontroller, sets the high
 * address, send (block-by-block) the binary to the microcontroller,
 * its CRC. At end, it closes the programming session and download
 * the calculated and stored CRCs to compare them to the local CRC
 * (read from .crc file)
 *
 * It manages also a timer in order to follow time to flash
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] binFilename path to the firmware binary to flash
 * @param[in] crcFilename path to the CRC associated to the firmware binary
 * @param[in] prog_trk (GUI) structure used to follow a firmware update
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_write_app(const unsigned int pnp,
		char const * const binFilename,
		char const * const crcFilename,
		progress_tracker_t *prog_trk)
{
	int ret = -1;
	/* Binary file descriptor */
	FILE *binfd;
	/* Size of a CAN frame (2 bytes for address, 6 for data) */
	unsigned char msg[8];
	int getDataExitCode = 0;
	unsigned int highAddr = 0, size = 0;
	unsigned int addr = APP_BASE_ADDR;
	struct timeval tv_begin, tv_end;
	char *str_time;
	/* CRC read from file */
	unsigned int crc = 0;
	/* For progression print */
	struct upload_percent percent;

	binfd = fopen(binFilename, "rb");
	if (binfd == NULL) {
		log_submit(eMessageTypeError, "fopen() failed on file %s (error "
				"code = %d)", binFilename, errno);
		goto out;
	}

	/* Get CRC from .crc file */
	ret = wrapper_can_read_crc(crcFilename);
	if (ret == -1) {
		log_submit(eMessageTypeError, "Couldn't read CRC from file %s",
				crcFilename);
		goto out_binfd;
	}
	crc = ret;

	/* Get size of microcontroller binary */
	fseek(binfd, 0L, SEEK_END);
	size = ftell(binfd);
	/* Go back to file begin */
	fseek(binfd, 0L, SEEK_SET);

	/*
	 * Ask to microcontroller to compute and send app CRC. We can with this
	 * detect if microcontroller firmware is different from the one to download.
	 * But it has also an interest when microcontroller version to download and
	 * current one are the same. If the current microcontroller is corrupted,
	 * its generated CRC will be different. So we will flash
	 */
	ret = bsl_get_crc(pnp, eCanCrcFromCalculation, crc);
	if (ret == -1) {
		log_submit(eMessageTypeError, "Failed to get calculated app CRC");
		goto out_binfd;
	}
	/* Reset to default value (modified by bsl_get_crc() function) */
	ret = -1;

	/* Open programming session */
	if (bsl_open_prog_session(pnp) != 0) {
		log_submit(eMessageTypeError, "Failed to open programming session");
		goto out_binfd;
	}
	/* Set high addr to APP_LAST_ADDR */
	if (bsl_set_high_addr(pnp, highAddr) != 0) {
		log_submit(eMessageTypeError, "Failed to set high addr %u", highAddr);
		goto out_close_session;
	}

	/* Initialize a percentage structure */
	log_init_percent_struct(&percent, size, SIZE_OF_CAN_FRAME_DATA);

	/* Start timing */
	if (gettimeofday(&tv_begin, NULL) == -1) {
		log_submit(eMessageTypeError, "Unable to get time\ngettimeofday: ");
		goto out_close_session;
	}

	do {
		/* Set high addr */
		if (highAddr != (addr & 0xFFFF0000) >> 16) {
			highAddr = (addr & 0xFFFF0000) >> 16;

			if (bsl_set_high_addr(pnp, highAddr) != 0) {
				log_submit(eMessageTypeError, "Failed to set high "
						"addr %u", highAddr);
				goto out_close_session;
			}
		}

		/* Get the 6 next bytes of the binary app */
		getDataExitCode = get_data_from_file(binfd, msg + 2);
		if (getDataExitCode == -1) {
			log_submit(eMessageTypeError, "Failed to get data from file %s "
					"(error code = %d)", binFilename, getDataExitCode);
			goto out_close_session;
		}

		percent.counter++;

		/* Write low addr in the CAN data */
		msg[0] = addr & 0xFF;
		msg[1] = (addr & 0xFF00) >> 8;

		/* Write low addr and app data */
		ret = bsl_write_program(pnp, msg);
		if (ret != 0) {
			if (ret == -1)
				log_submit(eMessageTypeError, "Program write failed at address "
						"[high:0x%.2X%.2X low:%.2X%.2X]", can_ctxt.high_addr[0],
						can_ctxt.high_addr[1], can_ctxt.low_addr[0],
						can_ctxt.low_addr[1]);
			else if (ret == -2)
				log_submit(eMessageTypeWarning, "Waiting cancelled by user "
						"(terminating signal sent)");

			goto out_close_session;
		}
		/* Reset to default value (modified by bsl_write_program() function) */
		ret = -1;

		/* Increase uC memory address */
		addr += SIZE_OF_CAN_FRAME_DATA;

		/* If we have sent enough blocks to print a message */
		if (percent.counter == percent.threshold) {
			/* Reinitialize counter */
			percent.counter = 0;
			percent.cur_percent += percent.slice_percent;

			/*
			 * Do not print when we are at 100%, because the mathematics formula
			 * is not very precise (there are still blocks to send)
			 */
			if (percent.cur_percent < 100) {
				fprintf(stderr, "[%.2d%%] Uploaded part at address: [high:0x%.2X%.2X "
						"low:%.2X%.2X]\n", percent.cur_percent, can_ctxt.high_addr[0],
						can_ctxt.high_addr[1], can_ctxt.low_addr[0], can_ctxt.low_addr[1]);
				if (prog_trk != NULL) {
					notify_progress(prog_trk, percent.cur_percent);
				}
			}
		}
	} while ((getDataExitCode != END_OF_FILE) && (addr <= APP_LAST_ADDR));

	/* File size to upload is out of microcontroller address space */
	if (getDataExitCode != END_OF_FILE) {
		log_submit(eMessageTypeError, "Program is too big !");
		goto out_close_session;
	}

	/* Stop timing */
	if (gettimeofday(&tv_end, NULL) == -1) {
		log_submit(eMessageTypeError, "Unable to get time\ngettimeofday: ");
		goto out_close_session;
	}
	if ((str_time = process_time_tostr(tv_begin, tv_end)) == NULL) {
		log_submit(eMessageTypeError, "Unable to get elapsed time to upload "
				"files %s and %s", binFilename, crcFilename);
		goto out_close_session;
	}
	log_submit(eMessageTypeNote, "[100%%] Uploading file %s done. Elapsed "
			"time: %s", binFilename, str_time);
	free(str_time);

	/*
	 * CRC is stored at uC high address CRC_HIGH_ADDR. If the current
	 * high address is different, set to CRC_HIGH_ADDR
	 */
	if ((can_ctxt.high_addr[0] | (can_ctxt.high_addr[1] << 8)) != CRC_HIGH_ADDR) {
		if (bsl_set_high_addr(pnp, CRC_HIGH_ADDR) != 0) {
			log_submit(eMessageTypeError, "Failed to set high addr %u",
					highAddr);
			goto out_close_session;
		}
	}

	/* Write CRC */
	msg[0] = (CRC_LOW_ADDR & 0xFF);
	msg[1] = (CRC_LOW_ADDR & 0xFF00) >> 8;
	/* Write CRC read from file to a frame */
	msg[2] = crc & 0xFF;
	msg[3] = (crc & 0xFF00) >> 8;

	ret = bsl_write_program(pnp, msg);
	if (ret != 0) {
		log_submit(eMessageTypeWarning, ret == -2 ? "Waiting cancelled by user "
				"(terminating signal sent)" : "Failed to write CRC");

		goto out_close_session;
	}

	/* Success */
	ret = 0;

out_close_session:
	/* Close the programming session, even if we had an error */
	if (bsl_close_prog_session(pnp) != 0) {
		log_submit(eMessageTypeError, "Failed to close programming session");
		ret = -1;
	}
	/* We had somewhere an error */
	if (ret != 0)
		goto out_binfd;

	/* Check if CRC stored in uC memory matches */
	ret = bsl_get_crc(pnp, eCanCrcFromMemory, crc);
	switch (ret) {
		case 0:
			log_submit(eMessageTypeNote, "CRC from uC memory matches");
			break;
		case 1:
			log_submit(eMessageTypeError, "CRC from uC memory doesn't "
					"match (maybe altered)");
			ret = -1;
			goto out_binfd;
			break;
		case -1:
			log_submit(eMessageTypeError, "Failed to get app CRC from "
					"uC memory");
			goto out_binfd;
			break;
	}

	/* Check if CRC calculated by uC matches */
	ret = bsl_get_crc(pnp, eCanCrcFromCalculation, crc);
	switch (ret) {
		case 0:
			log_submit(eMessageTypeNote, "CRC calculated by uC matches");
			break;
		case 1:
			log_submit(eMessageTypeError, "CRC calculated by uC doesn't "
					"match (maybe altered). Retry to be sure");
			/* Check if CRC calculated by uC matches */
			ret = bsl_get_crc(pnp, eCanCrcFromCalculation, crc);
			if (ret == 0) {
				log_submit(eMessageTypeNote, "CRC calculated by uC matches");
				break;
			} else if (ret == 1) {
				log_submit(eMessageTypeError, "CRC calculated by uC "
						"doesn't match (maybe altered)");
				ret = -1;
			} else if (ret == -1) {
				log_submit(eMessageTypeError, "Failed to get app CRC "
						"calculated by Uc");
			}
			goto out_binfd;
			break;
		case -1:
			log_submit(eMessageTypeError, "Failed to get app CRC "
					"calculated by Uc");
			goto out_binfd;
			break;
	}

out_binfd:
	/* Close binary file */
	fclose(binfd);
out:
	return ret;
}

/**
 * @brief OPLK event callback
 * 
 * Callback registered in the OPLK wrapper for CAN part. This function will
 * be called when interesting events happens in the OPLK wrapper. It must
 * respect this prototype to be a valid callback
 *
 * @param[in] evt the event which appended
 * @param[in] data the event data associated. Mostly NULL, check
 * eventCallback.h for details
 */
void wrapper_can_events_callback(const oplkEventType evt, const void *data)
{
	fprintf(stderr, "Got an oplk event: '%s'\n",
			debugstr_getOplkWrapperEventStr(evt));

	if (evt == OPLK_WRAPPER_SDO_TRANSFER_OK ||
			evt == OPLK_WRAPPER_SDO_TRANSFER_FAILED) {
		const tSdoComFinished *sdoComFinished = (const tSdoComFinished *)data;
		fprintf(stderr, "SDO transfert finished : nodeId:%u (0x%X/%u) "
				"abordCode:%d transferredBytes:%u\n",
				sdoComFinished->nodeId,
				sdoComFinished->targetIndex,
				sdoComFinished->targetSubIndex,
				sdoComFinished->abortCode,
				sdoComFinished->transferredBytes);

		wrapper_SDO_ctxt.sent = sdoComFinished->transferredBytes;
		if (evt == OPLK_WRAPPER_SDO_TRANSFER_FAILED) {
			wrapper_SDO_ctxt.error = sdoComFinished->abortCode;
		}

		pthread_cond_signal(&condStackOff);
	} else if (evt == OPLK_WRAPPER_NODES_SYNCED_OK ||
			evt == OPLK_WRAPPER_NODES_SYNCED_FAILED) {
		pthread_cond_signal(&condStackOff);
	} else if (evt == OPLK_STACK_NMT) {
		fprintf(stderr, "NMT state for node %d : \"%s\" (0x%04x)\n",
				((const tOplkApiEventNode *)data)->nodeId,
				debugstr_getOplkWrapperNmtStateStr(((const tOplkApiEventNode *)data)->nmtState),
				((const tOplkApiEventNode *)data)->nmtState);
	}
}

/**
 * @brief Initialize the shared CAN context
 *
 * Initialize the struct shared between FwUpdater and PDO
 * callback with default values
 */
void wrapper_initialize_can_context(void)
{
	/* Frame buffers Initialized to zero */
	can_ctxt.input.frame =
		(struct s_canFrame) { .id = 0, .dlc = 0, .data = { 0 } };
	can_ctxt.output.frame =
		(struct s_canFrame) { .id = 0, .dlc = 0, .data = { 0 } };

	/* No (in|out)coming frame stored */
	can_ctxt.input.frame_stored = false;
	can_ctxt.output.frame_stored = false;

	can_ctxt.input.mutex = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
	can_ctxt.output.condStackOff = (pthread_cond_t) PTHREAD_COND_INITIALIZER;

	/* Disable frame reception */
	can_ctxt.expected_id = CAN_DISABLED_INCOMING;

	can_ctxt.high_addr[0] = 0x0;
	can_ctxt.high_addr[1] = 0x0;

	can_ctxt.low_addr[0] = 0x0;
	can_ctxt.low_addr[1] = 0x0;
}

/** 
 * @brief Initialize the Openpowerlink stack (CAN version)
 *
 * Initialize the Openpowerlink stack with some default values, the defined
 * 'mnobd.cdc' file and a cycle
 *
 * Inspired by the one in wrapper_oplk.c
 *
 * @param[in] mnobd Path to the 'mnobd.cdc' file to use
 * @param[in] cycle Cycle (in milliseconds) to apply to the stack
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_can_initialize_oplk(char *mnobd, int cycle)
{
	int ret = 0;
	wOplkInitParam oplkParams;
	tOplkError oplkRet;

	memset(&oplkParams, 0, sizeof(wOplkInitParam));
	/* 0 = use the one from the cdc file */
	oplkParams.cycleLenUs              = cycle;
	oplkParams.processImageInSize      = sizeof(s_canPiIn);
	oplkParams.processImageOutSize     = sizeof(s_canPiOut);
	oplkParams.cdcFilename             = mnobd;
	oplkParams.callbackEvent           = wrapper_can_events_callback;
	oplkParams.callbackProcessSync     = wrapper_can_process_pdo;
	oplkParams.multiplCylceCnt         = 0;

	/* Initialize CAN context */
	wrapper_initialize_can_context();

	static const int MAX_TRIES_NB = 2;

	log_submit(eMessageTypeNote, "Start OpenPowerlink wrapper "
			"(for CAN) with .cdc file %s", mnobd);
	/* OPLK wrapper start has several tries */
	for (int try = 0; try < MAX_TRIES_NB; try++) {
		oplkRet = wOplkInit(&oplkParams);
		/* Error in OpenPowerlink initialization */
		if (oplkRet != kErrorOk) {
			if (oplkRet == kErrorNoResource)
				log_submit(eMessageTypeError, "An error happend while starting "
						"wrapper oplk : \"%s\" (0x%04x)\nMaybe OpenPowerlink "
						"nodes are not wired, or another process currently "
						"use network",
						debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
			else
				log_submit(eMessageTypeError, "An error happend while starting "
						"wrapper oplk : \"%s\" (0x%04x)",
						debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);

			/* OPLK init failed, try to shutdown previous OPLK execution */
			log_submit(eMessageTypeError, "Trying to shutdown previous "
					"OpenPowerlink execution");
			oplkRet = shutdownPowerlink();
			if (oplkRet != kErrorOk) {
				log_submit(eMessageTypeError,
						"Error in shutdown of OpenPowerLink wrapper : \"%s\" "
						"(0x%04x)",
						debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
				ret = -1;
				break;
			} else {
				log_submit(eMessageTypeError, "Previous OpenPowerlink shutdown "
						"successfully, retry launching");
			}
		} else {
			/*
			 * Wait some time to let the stack
			 * find & sync controlled nodes
			 */
			log_submit(eMessageTypeNote, "Wait OpenPowerlink nodes finding "
					"and synchronisation");
			if (wrapper_wait_time(&condStackOff, &mutex, 15) == -2) {
				ret = -2;
				/* User asked to stop FwUpdater, so stop also OpenPowerlink */
				shutdownPowerlink();
			}
			/* OPLK init succeeded, exit loop */
			break;
		}
	}

	return ret;
}

/**
 * @brief Detect existing microcontrollers, their firmware mode and version
 *
 * Wait for full detection of CAN/microcontrollers nodes, then check
 * for each node its applicative version with defined nodes in XML file
 *
 * Note: unlike others scanning functions (OPLK and PCIe), we don't have
 * a "verify" parameter, because version checks are done directly after
 * flash
 *
 * @param[in] hnlist hardware nodes list
 * @param[in] prog_trk (GUI) structure used to follow a firmware update
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_can_scan_bus(struct hardware_node_list *hnlist,
		progress_tracker_t *prog_trk)
{
	int ret, i, j, cmp_version;
	e_canReturnCode retcan = CAN_NODE_OK;
	struct can_mc_metadata {
		int max_mc_nodes;
		unsigned int const *id_mc_array;
		e_canCard card_type;
	} mc_meta;
	/* 21 is to comply with Stago versions */
	char version[21];

	/* For each user defined node */
	for (i = 0; i < hnlist->count ; i++) {
		/* If defined node is not a CAN node (microcontroller) */
		if (hnlist->node[i].bustype != eBusTypeCAN)
			continue;

		/* If node shouln't to be flashed */
		if (!hnlist->node[i].to_update)
			continue;

		if (strequ(hnlist->node[i].name, "APFB")) {
			mc_meta.max_mc_nodes = NB_APFB_NODES;
			mc_meta.id_mc_array = APFB_CARD_IDS;
			mc_meta.card_type = e_canCardAPFB;
		} else if (strequ(hnlist->node[i].name, "HIL")) {
			mc_meta.max_mc_nodes = NB_HIL_NODES;
			mc_meta.id_mc_array = HIL_CARD_IDS;
			mc_meta.card_type = e_canCardHIL;
		} else if (strequ(hnlist->node[i].name, "LLDCAN")) {
			mc_meta.max_mc_nodes = NB_LLD_NODES;
			mc_meta.id_mc_array = LLD_CARD_IDS;
			mc_meta.card_type = e_canCardLLD;
		} else {
			log_submit(eMessageTypeError, "Unknown microcontroller "
					"(%s)", hnlist->node[i].name);
			return -1;
		}

		/* For each APFB/HIL/LLD card */
		for (j = 0 ; j < mc_meta.max_mc_nodes ; j++) {
			/*
			 * Detect state of microcontroller subnode
			 * (applicative or factory)
			 */
			retcan = wrapper_can_detect_node(mc_meta.card_type,
					mc_meta.id_mc_array[j]);
			switch (retcan) {

				case CAN_NODE_UNDETECTED:
					log_submit(eMessageTypeError, "Undetected microcontroller: "
							"[name:%s pnp:%d]", hnlist->node[i].name,
							mc_meta.id_mc_array[j]);
					return -1;
					break;

				case CAN_NODE_IN_BSL:
					/* Get factory version of the subnode */
					ret = bsl_get_version(mc_meta.id_mc_array[j], version);
					if (ret != 0) {
						if (ret == -1)
							log_submit(eMessageTypeError, "Couldn't get "
									"factory version of microcontroller: "
									"[name:%s pnp:%d]", hnlist->node[i].name,
									mc_meta.id_mc_array[j]);

						return ret;
					}

					log_submit(eMessageTypeNote, "Found subnode: [name:%s "
							"pnp:%d factory_version:%s] -> is already in "
							"factory mode, upgrade to [version:%s]",
							hnlist->node[i].name, mc_meta.id_mc_array[j],
							version, hnlist->node[i].version);
					/* Subnode has to be flashed */
					hnlist->node[i].mc_subnode[j].to_update = true;
					/* Subnode is in factory mode */
					hnlist->node[i].mc_subnode[j].is_factory_mode = true;
					break;

				case CAN_NODE_IN_APP:
					/* Get version of the subnode */
					ret = app_get_version(mc_meta.card_type,
							mc_meta.id_mc_array[j], version);
					if (ret != 0) {
						if (ret == -1)
							log_submit(eMessageTypeError, "Couldn't get "
									"version of microcontroller: [name:%s "
									"pnp:%d]", hnlist->node[i].name,
									mc_meta.id_mc_array[j]);

						return ret;
					}

					/*
					 * Compare software version between XML version
					 * and got version (in OpenPowerlink node)
					 */
					cmp_version = context_compare_version(
							hnlist->node[i].version, version);
					switch(cmp_version) {
						/*
						 * If XML defined version of node is equals to
						 * got version, useless to upgrade firmware
						 */
						case 0:
							/* Disable firmware subnode updating */
							hnlist->node[i].mc_subnode[j].to_update = false;

							log_submit(eMessageTypeNote, "Found subnode: [name:"
									"%s pnp:%d version:%s] -> no need to flash "
									"(same version)", hnlist->node[i].name,
									mc_meta.id_mc_array[j], version);
							break;
						case 1:
							log_submit(eMessageTypeNote, "Found subnode: [name:"
									"%s pnp:%d version:%s] -> upgrade to "
									"[version:%s]", hnlist->node[i].name,
									mc_meta.id_mc_array[j], version,
									hnlist->node[i].version);
							break;
						case 2:
							log_submit(eMessageTypeNote, "Found subnode: [name:"
									"%s pnp:%d version:%s] -> downgrade to "
									"[version:%s]", hnlist->node[i].name,
									mc_meta.id_mc_array[j], version,
									hnlist->node[i].version);
							break;
						case -1:
							log_submit(eMessageTypeError, "Bad software version"
									" format (%s) for subnode [name:%s pnp:%d]",
									version, hnlist->node[i].name,
									mc_meta.id_mc_array[j]);
							return -1;
							break;
					}
					break;

				default:
					log_submit(eMessageTypeError, "Unknow state : %d", retcan);
					return -1;
					break;
			}
		}

		/*
		 * Tip: we officialy need to flash all occurences of a microcontroller,
		 * but maybe in fact we have previously disabled their update (because
		 * they are already up to date). So we can disable the node itself
		 */
		if (mc_meta.max_mc_nodes == 2) {
			/* Both submodules (usually of APFB/LLD) don't need to be flashed */
			if ((hnlist->node[i].mc_subnode[0].to_update == false) &&
					(hnlist->node[i].mc_subnode[1].to_update == false))
				hnlist->node[i].to_update = false;
		} else {
			/* The submodule (usually the HIL) doesn't need to be flashed */
			if (hnlist->node[i].mc_subnode[0].to_update == false)
				hnlist->node[i].to_update = false;
		}
		if (prog_trk != NULL && hnlist->node[i].to_update == false) {
			prog_trk->package = hnlist->node[i].archive;
			notify_no_need(prog_trk);
		}
	}
	return 0;
}

/** 
 * @brief Callback function processing PDOs (in CAN context)
 *
 * Like the PDO handler of OpenPowerlink wrapper, but manages also CAN frame
 * sending/receving in reading and writing Openpowerlink PDO.
 *
 * Called by OpenPowerlink stack
 *
 * @param[in] piOut Area memory to write (FwUpdater -> slaveboard)
 * @param[in] piIn Area memory to read (slaveboard -> FwUpdater)
 * @param[in] sizeIn Sizeof of input area memory
 * @param[in] sizeOut Sizeof of output area memory
 */
void wrapper_can_process_pdo(void* piOut, const void* piIn,
		uint32_t sizeIn, uint32_t sizeOut,
		struct timeval* const currentCycleTv,
		struct timeval* const nextCycleTv)
{
	if (wrapper_PDO_ctxt.send_cmd_word == true) {
		/* No new command word to send for now */
		wrapper_PDO_ctxt.send_cmd_word = false;
		/* Write command word in PDO */
		((PI_IN*)piOut)->s_TPDO_generalsystem[0] = wrapper_PDO_ctxt.cmd_word;
		/* A return code is expected */
		wrapper_PDO_ctxt.get_cmd_ret = true;
		log_submit(eMessageTypeNote, "PDO command : \"%s\" (0x%X)",
				wrapper_get_command_word_str(wrapper_PDO_ctxt.cmd_word),
				wrapper_PDO_ctxt.cmd_word);
	} else if (wrapper_PDO_ctxt.get_cmd_ret == true) {
		/*
		 * Return command have same high 16 bits than command
		 * itself. So checks if there are those same bits
		 */
		if ((((PI_OUT*)piIn)->s_RPDO_generalsystem[0] & BSL_CMD_FIELD_MSK) == wrapper_PDO_ctxt.cmd_word) {
			log_submit(eMessageTypeNote, "PDO return : \"%s\" (0x%X)",
					wrapper_get_command_return_str(((PI_OUT*)piIn)->s_RPDO_generalsystem[0] & BSL_STAT_ERR_CODE_MSK),
					((PI_OUT*)piIn)->s_RPDO_generalsystem[0] & BSL_STAT_ERR_CODE_MSK);
			/* If there is error (BSL_STAT_ERR_CODE_MSK bits different from 0) */
			if ((((PI_OUT*)piIn)->s_RPDO_generalsystem[0] & BSL_STAT_ERR_CODE_MSK) != 0) {
				wrapper_PDO_ctxt.error = true;
			}
			wrapper_store_PDO_command_return(&wrapper_PDO_ctxt,
					((PI_OUT*)piIn)->s_RPDO_generalsystem[0]);
		/*
		 * High 16 bits of return command doesn't match,
		 * probably because command is refused. Checks
		 * in corresponding bits if it is the case
		 */
		} else if ((((PI_OUT*)piIn)->s_RPDO_generalsystem[0] & BSL_STAT_CMD_REFUSED) == BSL_STAT_CMD_REFUSED) {
			log_submit(eMessageTypeError, "PDO return : \"Command refused\" "
					"(0x%X)", ((PI_OUT*)piIn)->s_RPDO_generalsystem[0]);
			wrapper_PDO_ctxt.error = true;
			wrapper_store_PDO_command_return(&wrapper_PDO_ctxt,
					((PI_OUT*)piIn)->s_RPDO_generalsystem[0]);
		}
	}

	(void)sizeIn;
	(void)sizeOut;
	(void)*currentCycleTv;
	(void)*nextCycleTv;

	/* Check if the last frame has been sent on the CAN bus by the fw */
	if (wrapper_can_outgoing_frame_ackd(&((s_canPiOut*)piOut)->controlWord,
				&((s_canPiIn*)piIn)->statusWord) == 1)
	{
		pthread_mutex_lock(&(can_ctxt.output.mutex));
		/* If there is a frame to send */
		if (can_ctxt.output.frame_stored) {
			can_ctxt.output.frame_stored = false;
			/* Copy local frame to PDO */
      ((s_canPiOut*)piOut)->outgoingFrameId = can_ctxt.output.frame.id;
      ((s_canPiOut*)piOut)->outgoingFrameLength = can_ctxt.output.frame.dlc;
      for (int i = 0; i < 8; i++)
				((s_canPiOut*)piOut)->outgoingFrameData[i] = can_ctxt.output.frame.data[i];
			/* Tell the FwUpdater that frame had been sent */
			pthread_cond_signal(&(can_ctxt.output.condStackOff));
			/* Notify the microcontroller that it has to handle a new frame */
      wrapper_can_notify_new_outgoing_frame(&((s_canPiOut*)piOut)->controlWord,
                                            &((s_canPiIn*)piIn)->statusWord);
		}
		pthread_mutex_unlock(&(can_ctxt.output.mutex));
  }

  /* Check if a new incoming frame needs to be processed */
  if (wrapper_can_new_frame_rcvd(&((s_canPiOut*)piOut)->controlWord,
			  &((s_canPiIn*)piIn)->statusWord))
  {
		pthread_mutex_lock(&(can_ctxt.input.mutex));
		if (((s_canPiIn*)piIn)->incomingFrameId == can_ctxt.expected_id) {
			/* If no incoming frame is stored */
			if (can_ctxt.input.frame_stored == false) {
				/* Copy frame from PDO to local buffer */
				can_ctxt.input.frame.id = ((s_canPiIn*)piIn)->incomingFrameId;
				can_ctxt.input.frame.dlc = ((s_canPiIn*)piIn)->incomingFrameLength;
				memcpy(can_ctxt.input.frame.data, ((s_canPiIn*)piIn)->incomingFrameData,
						sizeof(unsigned char) * 8);
				can_ctxt.input.frame_stored = true;
				/* Disable frame reception */
				can_ctxt.expected_id = CAN_DISABLED_INCOMING;
				pthread_cond_signal(&(can_ctxt.input.condStackOff));
			} else {
				log_submit(eMessageTypeError, "Input frame slot full "
						"(stored_id:0x%X id_to_store:0x%X)",
						can_ctxt.input.frame.id,
						((s_canPiIn*)piIn)->incomingFrameId);
			}
		} else if ((((((s_canPiIn*)piIn)->incomingFrameId) < 0x480) &&
				((((s_canPiIn*)piIn)->incomingFrameId) > 0x49F)) &&
				(((((s_canPiIn*)piIn)->incomingFrameId) < 0x700) &&
				((((s_canPiIn*)piIn)->incomingFrameId) > 0x758))) {
				log_submit(eMessageTypeWarning, "Dropped frame [id:0x%X dlc:"
						"0x%X data:0x%X %X %X %X %X %X %X %X]",
						((s_canPiIn*)piIn)->incomingFrameId,
						((s_canPiIn*)piIn)->incomingFrameLength,
						((s_canPiIn*)piIn)->incomingFrameData[0],
						((s_canPiIn*)piIn)->incomingFrameData[1],
						((s_canPiIn*)piIn)->incomingFrameData[2],
						((s_canPiIn*)piIn)->incomingFrameData[3],
						((s_canPiIn*)piIn)->incomingFrameData[4],
						((s_canPiIn*)piIn)->incomingFrameData[5],
						((s_canPiIn*)piIn)->incomingFrameData[6],
						((s_canPiIn*)piIn)->incomingFrameData[7]);
		}
		/* Notify the microcontroller that its frame has been received */
		wrapper_can_ack_incoming_frame(&((s_canPiOut*)piOut)->controlWord,
				&((s_canPiIn*)piIn)->statusWord);
		pthread_mutex_unlock(&(can_ctxt.input.mutex));
  }
}

/**
 * @brief Send frame and wait its return (a frame with expected id)
 *
 * Function that send a CAN frame and wait its return (with timeout). A check
 * is performed if response is a status frame
 *
 * @param[in] snd_frame CAN frame to send
 * @param[in,out] rcv_frame Response of sent frame
 * @param[in] expected CAN ID expected for the response
 * @param[in] timeout timeout (in seconds) for both send and receive time
 *
 * @return 0 if succeeded, -1 else
 */
static int send_frame_and_wait(s_canFrame snd_frame, s_canFrame *rcv_frame,
		int expected, int timeout) {
	int ret = 0;
	struct timespec ts;
	const char *err_str = NULL;

	/* Initialize buffer of received frame */
	memset(rcv_frame, 0, sizeof(*rcv_frame));

	/* Send frame */
	pthread_mutex_lock(&(can_ctxt.output.mutex));
	/* There is still a stored frame not sent to the microcontroller */
	while (can_ctxt.output.frame_stored == true) {
		/* Set a timeout of 5 seconds */
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += CAN_RESPONSE_TIMEOUT;

		/* Wait during 5 seconds for the right to send a frame */
		ret = pthread_cond_timedwait(&(can_ctxt.output.condStackOff),
				&(can_ctxt.output.mutex), &ts);
		if (ret == ETIMEDOUT) {
			log_submit(eMessageTypeError, "Timeout to send frame 0x%X",
					snd_frame.id);
			pthread_mutex_unlock(&(can_ctxt.output.mutex));
			return -1;
		}
	}
	/* Copy frame to send */
	memcpy(&(can_ctxt.output.frame), &(snd_frame), sizeof(s_canFrame));
	can_ctxt.output.frame_stored = true;
	/*
	 * We use incoming mutex because we want access to the
	 * 'can_ctxt.expected_id' variable (acceded by incoming part)
	 */
	pthread_mutex_lock(&(can_ctxt.input.mutex));
	can_ctxt.expected_id = expected;
	pthread_mutex_unlock(&(can_ctxt.input.mutex));
	pthread_mutex_unlock(&(can_ctxt.output.mutex));

	pthread_mutex_lock(&(can_ctxt.input.mutex));
	while (can_ctxt.input.frame_stored == false) {
		/* Set a timeout of 5 seconds */
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += timeout;
		/* Wait during some seconds the reception of expected frame */
		ret = pthread_cond_timedwait(&(can_ctxt.input.condStackOff),
				&(can_ctxt.input.mutex), &ts);
		if (ret == ETIMEDOUT) {
			log_submit(eMessageTypeError, "Timeout to receive frame 0x%X",
					expected);
			/* Disable frame input and clean input buffer */
			can_ctxt.expected_id = CAN_DISABLED_INCOMING;
			memset(&(can_ctxt.input.frame), 0, sizeof(s_canFrame));
			pthread_mutex_unlock(&(can_ctxt.input.mutex));
			return -1;
		}
	}
	memcpy(rcv_frame, &(can_ctxt.input.frame), sizeof(*rcv_frame));
	can_ctxt.input.frame_stored = false;
	/* Disable frame receiving */
	can_ctxt.expected_id = CAN_DISABLED_INCOMING;
	pthread_mutex_unlock(&(can_ctxt.input.mutex));

	/* Received frame is a status frame */
	if (((rcv_frame->id) & (CAN_ID_BSL_STATUS)) == CAN_ID_BSL_STATUS) {
		/* Error in status byte */
		if (rcv_frame->data[0] > 1) {
			/* Get a representative string of error(s) */
			err_str = wrapper_can_get_status_return_str(rcv_frame->data[0]);
			log_submit(eMessageTypeError, "Status frame 0x%X error(s): "
					"\"%s\" (0x%X)", expected, err_str, rcv_frame->data[0]);
			free((char*)err_str);
			log_submit(eMessageTypeError, "Sent frame trace: [id:0x%X dlc:0x%X "
					"data:0x%.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X]",
					snd_frame.id, snd_frame.dlc, snd_frame.data[0],
					snd_frame.data[1], snd_frame.data[2], snd_frame.data[3],
					snd_frame.data[4], snd_frame.data[5], snd_frame.data[6],
					snd_frame.data[7]);
			return -1;
		}
		/*
		 * Status frame always returns the high address. If
		 * different, there was a problem
		 */
		else if ((rcv_frame->data[4] != can_ctxt.high_addr[0]) ||
				(rcv_frame->data[5] != can_ctxt.high_addr[1])) {
			log_submit(eMessageTypeError, "Status frame returned bad high "
					"address (0x%.2X %.2X), expected: 0x%.2X %.2X",
					rcv_frame->data[4], rcv_frame->data[5],
					can_ctxt.high_addr[0],
					can_ctxt.high_addr[1]);
			return -1;
		} else if ((rcv_frame->data[2] != can_ctxt.low_addr[0]) ||
				(rcv_frame->data[3] != can_ctxt.low_addr[1])) {
			log_submit(eMessageTypeError, "Status frame returned bad low "
					"address (0x%.2X %.2X), expected: 0x%.2X %.2X",
					rcv_frame->data[2], rcv_frame->data[3],
					can_ctxt.low_addr[0],
					can_ctxt.low_addr[1]);
			return -1;
		}
	}

	return 0;
}

/**
 * @brief Check if the FwUpdater/PDO handler can emit a new frame
 *
 * This function checks some bits to know if the OUTACK
 * bit of status word has changed
 *
 * @param[in] control_word Control word (FwUpdater side)
 * @param[in] status_word Status word (slaveboard side)
 *
 * @return 0 if succeeded, 1 else
 */
static int wrapper_can_outgoing_frame_ackd(
		unsigned short const * const control_word,
		unsigned short const * const status_word)
{
	int ret = 0;

	if (((*control_word & CAN_OUTRDY_BIT) >> CAN_OUTRDY_LSH) ==
			((*status_word & CAN_OUTACK_BIT) >> CAN_OUTACK_LSH))
		ret = 1;

	return ret;
}

/**
 * @brief Notify the firmware that a new TX frame is available
 *
 * Modify the control word that, when sent by PDO with a CAN frame, will
 * alert slaveboard that there is a new frame.
 *
 * @param[in] control_word Control word (FwUpdater side)
 * @param[in] status_word Status word (slaveboard side)
 */
static void wrapper_can_notify_new_outgoing_frame(
		unsigned short * const control_word,
		unsigned short const * const status_word)
{
	/*
	 * To notify the firmware CAN_OUTRDY_BIT must
	 * be the opposite of CAN_OUTACK_BIT
	 */
	if (*status_word & CAN_OUTACK_BIT)
		*control_word &= ~CAN_OUTRDY_BIT;
	else
		*control_word |= CAN_OUTRDY_BIT;
}

/**
 * @brief Check if a new RX frame from the CAN bus is available
 *
 * This function checks some bits to know if the INACK bit of status word
 * has changed (meaning new frame from CAN bus)
 *
 * @param[in] control_word Control word (FwUpdater side)
 * @param[in] status_word Status word (slaveboard side)
 *
 * @return 0 if succeeded, 1 else
 */
static int wrapper_can_new_frame_rcvd(unsigned short const * const control_word,
		unsigned short const * const status_word)
{
	int ret = 0;

	if ((*control_word & CAN_INACK_BIT) >> CAN_INACK_LSH !=
			(*status_word & CAN_INRDY_BIT) >> CAN_INRDY_LSH)
		ret = 1;

	return ret;
}

/**
 * @brief Acknowledge the RX frame received from the CAN bus
 *
 * Modify the control word that, when sent by PDO with a CAN frame, will
 * alert slaveboard that its frame has been received (acknowledgement)
 *
 * @param[in] control_word Control word (FwUpdater side)
 * @param[in] status_word Status word (slaveboard side)
 */
static void wrapper_can_ack_incoming_frame(unsigned short * const control_word,
		unsigned short const * const status_word)
{
	if (*status_word & CAN_INRDY_BIT)
		*control_word |= CAN_INACK_BIT;
	else
		*control_word &= ~CAN_INACK_BIT;
}

/**
 * @brief Calculate a CAN ID
 *
 * Calculate a CAN ID according to the card type and the pnp value. Function
 * usually used in APP mode context, not for BSL mode context (because the
 * BSL mode doesn't need specific card offset)
 *
 * @param[in] card type of microcontroller (APFB, HIL or LLD)
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] baseId base id (frame), increased by @card and @pnp
 *
 * @return A calculated CAN ID 
 */
static unsigned int wrapper_can_calculate_frame_id(const e_canCard card,
		const unsigned int pnp, const unsigned int baseId)
{
	unsigned int id = baseId;

	if (card == e_canCardLLD) {
		id += CAN_ID_LLD_OFFSET;
	} else if (card == e_canCardAPFB) {
		id += CAN_ID_APFB_OFFSET;
	} else if (card == e_canCardHIL) {
		/*
		 * For the HIL, its pnp in APP mode is equals to '1'. The array
		 * CAN_ID_HIL_OFFSET stores the pnp '5', but this last one is
		 * only for BSL mode. This why we bypass pnp by a static '1'
		 */
		id += CAN_ID_HIL_OFFSET;
		return (id + 1);
	}
	return (id + pnp);
}

/**
 * @brief Check the presence of a CAN node and its mode (BSL or applicative)
 *
 * Send a wakeup request to a CAN node in order to know if it is alive/present
 * and its firmware mode (BSL or applicative). It exists two wakeup requests:
 * one for applicative mode, other for BSL mode. If applicative wakeup request
 * fails, this function will try with BSL wakeup request. We use this order
 * because there are much chance to have a applicative firmware rather a BSL
 * firmware
 *
 * @param[in] card type of microcontroller (APFB, HIL or LLD)
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 *
 * @return A enumeration matching the CAN node state
 */
static e_canReturnCode wrapper_can_detect_node(const e_canCard card,
		const unsigned int pnp)
{
	int ret = 0;

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to request APP/BSL wakeup");

		log_submit(eMessageTypeNote, "Request APP wakeup for: [name:%s pnp:%u]",
				mcu_to_string(card), pnp);

		/* Send frame */
		ret = app_request_wakeup(card, pnp, 1);
		if (ret == 0)
			return CAN_NODE_IN_APP;

		log_submit(eMessageTypeNote, "Request BSL wakeup for: [name:%s pnp:%u]",
				mcu_to_string(card), pnp);

		/* Send frame */
		ret = bsl_request_wakeup(pnp, 1);
		if (ret == 0)
			return CAN_NODE_IN_BSL;
	}

	return CAN_NODE_UNDETECTED;
}

/**
 * @brief Get 6 bytes of data from the binary file
 *
 * Read 6 bytes from a file descriptor (of a firmware binary file). The file
 * offset is managed by fread() function (stream), so each call to this
 * function reads the 6 next bytes
 *
 * @param[in] fd File descriptor of firmware binary 
 * @param[in,out] data Buffer where to store read data
 *
 * @return 0 if succeeded, -1 else
 */
static int get_data_from_file(FILE* fd,
		unsigned char data[SIZE_OF_CAN_FRAME_DATA])
{
	int ret = 0;

	fread(data, 1, SIZE_OF_CAN_FRAME_DATA, fd);

	if (ferror(fd)) {
		log_submit(eMessageTypeError, "fread() failed with error: %d",
				ferror(fd));
		ret = -1;
	} else if (feof(fd)) {
		ret = END_OF_FILE;
	}

	return ret;
}

/**
 * @brief Read the .crc file and returns its content (a CRC)
 *
 * Read a file and extracts from it a string. Its character size is checked
 * then converted to a integer.
 *
 * @param[in] crc_file path of the .crc file to read
 *
 * @return Unsigned integer of the read CRC
 */
static unsigned int wrapper_can_read_crc(char const * const crc_file)
{
	int fd, ret;
	unsigned int crc = 0;
	char data[5] = { 0 };
	char *endptr = NULL;

	/* Last character to '\0' */
	data[4] = '\0';

	/* Open .crc file */
	ret = open(crc_file, O_RDONLY);
	if (ret == -1) {
		log_submit_perror(eMessageTypeError, "Couldn't open file %s\nopen: ",
				crc_file);
		return -1;
	}
	fd = ret;

	/* Read 4 characters (size of a CRC) */
	ret = read(fd, data, 4);
	if (ret == -1) {
		log_submit_perror(eMessageTypeError, "Couldn't read file %s\nread: ",
				crc_file);
		close(fd);
		return -1;
	/*
	 * If we don't read exactly four characters, or we read four characters
	 * but a newline is present (meaning there are fewer characters)
	 */
	} else if ((ret != 4) || strchr(data, '\n')) {
		log_submit(eMessageTypeError, "Malformed CRC from file %s", crc_file);
		close(fd);
		return -1;
	}

	close(fd);

	errno = 0;
	/* Convert CRC string to hexadecimal */
	crc = strtol(data, &endptr, 16);

	/* Check if conversion has succeeded */
	if ((errno != 0) && (crc == 0)) {
		log_submit_perror(eMessageTypeError, "Malformed CRC from file %s "
				"(read: %s)\nstrtol: ", crc_file, data);
		return -1;
	}

	if (endptr == data) {
		log_submit(eMessageTypeError, "Malformed CRC from file %s (read: %s)",
				crc_file, data);
		return -1;
	}

	return crc;
}

/**
 * @brief Request wakeup frame for CAN node in applicative mode
 *
 * Note: unlike other functions, this one take an argument to define
 * max retries allowed when a frame is not received. This is because
 * in microcontroller detection step, it is useless to retry N times
 * when the microcontroller is in fact in factory/BSL mode
 *
 * * Send: CAN_ID_WAKEUP_FRAME
 * * Receive : CAN_ID_WAKEUP_FRAME
 *
 * @param[in] card type of microcontroller (APFB, HIL or LLD)
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] max_retry max number of send/receive tries
 *
 * @return 0 if succeeded, -1 else
 */
static int app_request_wakeup(const e_canCard card, const unsigned int pnp,
		int max_retry)
{
	int ret = 0;
	int expected_id = wrapper_can_calculate_frame_id(card, pnp,
			CAN_ID_WAKEUP_FRAME);
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = expected_id;
	snd_frame.dlc = 16;

	for (int try = 0 ; try < max_retry ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to request APP wakeup");

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_REBOOT_TIMEOUT);
		if (ret != 0)
			continue;

		break;
	}
	return ret;
}

/**
 * @brief Reboot/switch in BSL mode
 *
 * Send a reboot frame in order to reboot firmware of a microcontroller
 * (currently in applicative mode) to BSL mode
 *
 * If reboot sending/receiving fails, requests BSL wakeup
 *
 * * Send: CAN_ID_REBOOT_IN_BSL
 * * Receive: CAN_ID_BSL_WAKEUP
 *
 * @param[in] card type of microcontroller (APFB, HIL or LLD)
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 *
 * @return 0 if succeeded, -1 else
 */
static int app_reboot_in_bsl(const e_canCard card, const unsigned int pnp)
{
	int ret = 0;
	int expected_id = CAN_ID_BSL_WAKEUP + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = wrapper_can_calculate_frame_id(card, pnp,
			CAN_ID_REBOOT_IN_BSL);
	snd_frame.dlc = 0;

	ret = send_frame_and_wait(snd_frame, &(rcv_frame), expected_id,
			CAN_REBOOT_TIMEOUT);
	if (ret == 0)
		return ret;

	/* Wakeup frame not received, so we send an new BSL wakeup */
	log_submit(eMessageTypeWarning, "BSL wakeup frame 0x%X from card %u, "
			"pnp %u not received, request a new BSL wakeup", expected_id,
			card, pnp);
	ret = 0;

	return bsl_request_wakeup(pnp, CAN_MAX_RETRIES);
}

/**
 * @brief Request wakeup frame for BSL card
 *
 * Allow to check present of a microcontroller in its BSL firmware
 *
 * Note: unlike other functions, this one take an argument to define
 * max retries allowed when a frame is not received. This is because
 * in microcontroller detection step, it is useless to retry N times
 * when the microcontroller is in fact in applicative mode
 *
 * * Send: CAN_ID_BSL_WAKEUP
 * * Receive: CAN_ID_BSL_WAKEUP
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] max_retry max number of send/receive tries
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_request_wakeup(const unsigned int pnp, int max_retry)
{
	int ret = 0;
	int expected_id = CAN_ID_BSL_WAKEUP + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = expected_id;
	snd_frame.dlc = 16;

	for (int try = 0 ; try < max_retry ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to request BSL wakeup");

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);
		if (ret != 0)
			continue;

		break;
	}
	return ret;
}

/**
 * @brief Set high addr for programming
 *
 * Send a frame containing high address to a microcontroller (in BSL mode).
 * The high address set in what memory area the BSL has to put data sent by
 * FwUpdater
 *
 * * Send: CAN_ID_BSL_ACTION (set high address)
 * * Receive: CAN_ID_BSL_STATUS
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] addr address to set as high address
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_set_high_addr(const int pnp, const unsigned int addr)
{
	int ret = 0;
	int expected_id = CAN_ID_BSL_STATUS + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = CAN_ID_BSL_ACTION + pnp;
	snd_frame.dlc = 3;
	snd_frame.data[0] = 0x4;
	snd_frame.data[1] = addr & 0xFF;
	snd_frame.data[2] = (addr & 0xFF00) >> 8;

	log_submit(eMessageTypeNote, "Set high addr to 0x%.2X%.2X",
			snd_frame.data[1], snd_frame.data[2]);

	/*
	 * Store high address in the global variable used to check
	 * high address contained in each received status frame
	 */
	can_ctxt.high_addr[0] = snd_frame.data[1];
	can_ctxt.high_addr[1] = snd_frame.data[2];

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to send frame 0x%X",
					snd_frame.id);

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);
		if (ret != 0)
			continue;

		break;
	}
	return ret;
}

/**
 * @brief Write 6 bytes of binary program into the uC app memory
 *
 * Send a part (6 bytes) of a applicative firmware binary to the
 * microcontroller. It sends also the low address in order to
 * set where (in microcontroller memory) the part should be
 * wrote
 *
 * * Send: CAN_ID_BSL_PROGRAM_APP
 * * Receive: CAN_ID_BSL_STATUS
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] data data (extracted from a firmware binary) to send
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_write_program(const int pnp, const unsigned char data[8])
{
	int ret = 0;
	int expected_id = CAN_ID_BSL_STATUS + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = CAN_ID_BSL_PROGRAM_APP + pnp;
	snd_frame.dlc = 8;
	memcpy(snd_frame.data, data, sizeof(snd_frame.data));

	/*
	 * Store low address in the global variable used to check
	 * low address contained in each received status frame
	 */
	can_ctxt.low_addr[0] = snd_frame.data[0];
	can_ctxt.low_addr[1] = snd_frame.data[1];

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to send data frame 0x%X",
					snd_frame.id);

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);

		/* (FwUpdater cancellation point) */
		if (fTermSignalReceived_l == true)
			return -2;

		if (ret != 0)
			continue;

		break;
	}
	return ret;
}

/**
 * @brief Programming validity confirmation
 *
 * Send a frame to finish firmware upload (confirmation)
 *
 * * Send: CAN_ID_BSL_ACTION (validity confirmation)
 * * Receive: CAN_ID_BSL_STATUS
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_prog_validity_confirm(const unsigned int pnp)
{
	int ret = 0;
	int expected_id = CAN_ID_BSL_STATUS + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = CAN_ID_BSL_ACTION + pnp;
	snd_frame.dlc = 3;
	snd_frame.data[0] = 7;
	snd_frame.data[1] = 0xA5;
	snd_frame.data[2] = 0;

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to send validity frame 0x%X",
					snd_frame.id);

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);
		if (ret != 0)
			continue;

		break;
	}
	return ret;
}

/**
 * @brief Open programming session
 *
 * Alert the microcontroller by a frame that it will receipt a new firmware
 *
 * * Send: CAN_ID_BSL_ACTION (open programming session)
 * * Receive: CAN_ID_BSL_STATUS
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_open_prog_session(const unsigned int pnp)
{
	int ret = 0;
	int expected_id = CAN_ID_BSL_STATUS + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = CAN_ID_BSL_ACTION + pnp;
	snd_frame.dlc = 3;
	snd_frame.data[0] = 1;
	snd_frame.data[1] = 0;
	snd_frame.data[2] = 0;

	/* Initialize high address to zero */
	can_ctxt.high_addr[0] = 0x0;
	can_ctxt.high_addr[1] = 0x0;
	/* Initialize low address to zero */
	can_ctxt.low_addr[0] = 0x0;
	can_ctxt.low_addr[1] = 0x0;

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to send open programming "
					"session frame 0x%X", snd_frame.id);

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);
		if (ret != 0)
			continue;

		/*
		 * When a programming session is opened, the bit 0 of the status
		 * frame is equal to '1'. If not, there was a problem
		 */
		if (rcv_frame.data[0] != 1) {
			log_submit(eMessageTypeError, "Programming session was not opened");
			ret = -1;
			continue;
		}
		break;
	}
	return ret;
}

/**
 * @brief Close programming session
 *
 * Tell to the microcontroller, by a frame, to leave the internal context
 * of firmware updating. Used after both successful and error cases
 *
 * * Send: CAN_ID_BSL_ACTION (close programming session)
 * * Receive: CAN_ID_BSL_STATUS
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_close_prog_session(const unsigned int pnp)
{
	int ret = 0;
	int expected_id = CAN_ID_BSL_STATUS + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = CAN_ID_BSL_ACTION + pnp;
	snd_frame.dlc = 3;
	snd_frame.data[0] = 6;
	snd_frame.data[1] = 0;
	snd_frame.data[2] = 0;

	/*
	 * Because high/low address returned by status frame is resetted
	 * to zero when we close programming session, we have to set the
	 * global variables to zero
	 */
	can_ctxt.high_addr[0] = 0x0;
	can_ctxt.high_addr[1] = 0x0;
	can_ctxt.low_addr[0] = 0x0;
	can_ctxt.low_addr[1] = 0x0;

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to send close programming "
					"session frame 0x%X", snd_frame.id);

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);
		if (ret != 0)
			continue;

		/*
		 * When a programming session is closed, the bit 0 of the status
		 * frame is equal to '0'. If not, there was a problem
		 */
		if (rcv_frame.data[0] != 0) {
			log_submit(eMessageTypeError, "Programming session was not closed");
			ret = -1;
			continue;
		}

		break;
	}
	return ret;
}

/**
 * @brief Get a string of the microcontroller applicative version (quadruplet)
 *
 * Ask to the microcontroller (in applicative firmware) to send back a version
 * frame. Integers are extracted and formatted in a string. The version is
 * a quadruplet:
 * * Major : 2 bytes
 * * Minor : 2 bytes
 * * Sub-minor : 2 bytes
 * * Teamcity build id : 2 bytes
 * 
 * * Send: CAN_ID_VERSION
 * * Receive: CAN_ID_VERSION
 *
 * @param[in] card type of microcontroller (APFB, HIL or LLD)
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in,out] str_version buffer where the string is written
 *
 * @return 0 if succeeded, -1 else
 */
static int app_get_version(const e_canCard card, const unsigned int pnp,
		char *str_version)
{
	int ret = 0;
	/* Microcontroller version quadruplet */
	struct can_version {
		short major;
		short minor;
		short subminor;
		short build;
	} version;
	int expected_id = wrapper_can_calculate_frame_id(card, pnp, CAN_ID_VERSION);
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = expected_id;
	/*
	 * 16 is a special DLC meaning that microcontroller has to answer with
	 * the same frame and some informations (like version here)
	 */
	snd_frame.dlc = 16;

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to request version");

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);

		/* (FwUpdater cancellation point) */
		if (fTermSignalReceived_l == true)
			return -2;

		if (ret != 0)
			continue;

		break;
	}

	/* We fail to get a version */
	if (ret != 0)
		return ret;

	/* Get version components */
	version.major = rcv_frame.data[0] | (rcv_frame.data[1] << 8);
	version.minor = rcv_frame.data[2] | (rcv_frame.data[3] << 8);
	version.subminor = rcv_frame.data[4] | (rcv_frame.data[5] << 8);
	version.build = rcv_frame.data[6] | (rcv_frame.data[7] << 8);

	/* String initialization */
	memset(str_version, '\0', sizeof(char) * 21);
	/* Create a string formatted by the microcontroller version */
	if (snprintf(str_version, 21, "%d.%d.%d.%d", version.major, version.minor,
				version.subminor, version.build) >= 21) {
		log_submit(eMessageTypeError, "Version too long");
		return -1;
	}

	return 0;
}

/**
 * @brief Get a string of the microcontroller BSL version (quadruplet)
 *
 * Ask to the microcontroller (in BSL firmware) to send back a version frame.
 * Integers are extracted and formatted in a string. The version is a
 * quadruplet:
 * * Major : 2 bytes
 * * Minor : 2 bytes
 * * Sub-minor : 2 bytes
 * * Teamcity build id : 2 bytes
 *
 * * Send: CAN_ID_BSL_VERSION
 * * Receive: CAN_ID_BSL_VERSION
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in,out] str_version buffer where the string is written
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_get_version(const unsigned int pnp, char *str_version)
{
	int ret = 0;
	/* Microcontroller version quadruplet */
	struct can_version {
		short major;
		short minor;
		short subminor;
		short build;
	} version;
	int expected_id = CAN_ID_BSL_VERSION + pnp;
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = expected_id;
	snd_frame.dlc = 16;

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to request version");

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);

		/* (FwUpdater cancellation point) */
		if (fTermSignalReceived_l == true)
			return -2;

		if (ret != 0)
			continue;

		break;
	}

	/* We fail to get a version */
	if (ret != 0)
		return ret;

	/* Get version components */
	version.major = rcv_frame.data[0] | (rcv_frame.data[1] << 8);
	version.minor = rcv_frame.data[2] | (rcv_frame.data[3] << 8);
	version.subminor = rcv_frame.data[4] | (rcv_frame.data[5] << 8);
	version.build = rcv_frame.data[6] | (rcv_frame.data[7] << 8);

	/* String initialization */
	memset(str_version, '\0', sizeof(char) * 21);
	/* Create a string formatted by the microcontroller version */
	if (snprintf(str_version, 21, "%d.%d.%d.%d", version.major, version.minor,
				version.subminor, version.build) >= 21) {
		log_submit(eMessageTypeError, "Version too long");
		return -1;
	}

	return 0;
}

/**
 * @brief Get CRC of the uC applicative memory (stored or computed)
 *
 * Ask to the microcontroller to send its CRC, either by :
 * * read the CRC stored in the uC memory
 * * compute from the uC app memory
 *
 * Both cases return by the status frame. Then compare
 * the CRC with the argument one
 *
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 * @param[in] from CRC origin (uC memory or calculated)
 * @param[in] crc CRC to compare
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_get_crc(const unsigned int pnp, const eCanCrcFrom from,
		const unsigned int crc)
{
	int ret = 0;
	int expected_id = (from == eCanCrcFromCalculation) ?
		(CAN_ID_BSL_STATUS + pnp) :
		(CAN_ID_BSL_ACTION + pnp);
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };
	/*
	 * CRC read from file, converted in 'unsigned char'
	 * array to ease code reading (CRC comparaison)
	 */
	unsigned char cmp_crc[2] = { (crc & 0xFF), ((crc & 0xFF00) >> 8) };
	unsigned char rcv_crc[2] = { 0 };

	snd_frame.id = CAN_ID_BSL_ACTION + pnp;
	snd_frame.dlc = 3;
	snd_frame.data[0] = (from == eCanCrcFromCalculation) ? 5 : 9;
	snd_frame.data[1] = 0;
	snd_frame.data[2] = 0;

	for (int try = 0 ; try < CAN_MAX_RETRIES ; try++) {
		/*
		 * "ret" variable is already set by previous loop, so we
		 * can test it to know if we have to print a warning
		 */
		if (ret != 0)
			log_submit(eMessageTypeWarning, "Retry to send CRC request frame "
					"0x%X", snd_frame.id);

		ret = send_frame_and_wait(snd_frame, &(rcv_frame),
				expected_id, CAN_RESPONSE_TIMEOUT);
		if (ret != 0)
			continue;

		/* Debug */
		log_submit(eMessageTypeNote, "Sent frame trace: [id:0x%X dlc:0x%X "
				"data:0x%X %X %X %X %X %X %X %X]", snd_frame.id, snd_frame.dlc,
				snd_frame.data[0], snd_frame.data[1], snd_frame.data[2]);
		log_submit(eMessageTypeNote, "Rcv frame trace: [id:0x%X dlc:0x%X "
				"data:0x%X %X %X %X %X %X %X %X]", rcv_frame.id, rcv_frame.dlc,
				rcv_frame.data[0], rcv_frame.data[1], rcv_frame.data[2],
				rcv_frame.data[3], rcv_frame.data[4], rcv_frame.data[5],
				rcv_frame.data[6], rcv_frame.data[7]);

		if (from == eCanCrcFromCalculation) {
			/*
			 * The fourth byte is set to '1' if CRC is calculated.
			 * If equal to 0, there was a problem
			 */
			if (rcv_frame.data[1] == 0) {
				log_submit(eMessageTypeError, "Malformed status frame "
						"containing calculated CRC. Status frame trace: "
						"[id:0x%X dlc:0x%X data:0x%X %X %X %X %X %X %X %X]",
						rcv_frame.id, rcv_frame.dlc, rcv_frame.data[0],
						rcv_frame.data[1], rcv_frame.data[2],
						rcv_frame.data[3], rcv_frame.data[4],
						rcv_frame.data[5], rcv_frame.data[6],
						rcv_frame.data[7]);
				ret = -1;
				continue;
			}
			rcv_crc[0] = rcv_frame.data[6];
			rcv_crc[1] = rcv_frame.data[7];
		} else {
			/*
			 * Note: CRC sent by microcontroller is not in the same
			 * endianness (if CRC comes from memory only !)
			 */
			rcv_crc[0] = rcv_frame.data[2];
			rcv_crc[1] = rcv_frame.data[1];
		}

		break;
	}

	/* We fail to get a version */
	if (ret != 0)
		return ret;

	/* If CRC matches */
	if ((rcv_crc[0] == cmp_crc[0]) && (rcv_crc[1] == cmp_crc[1])) {
		log_submit(eMessageTypeNote, "Same CRC values (0x%X 0x%X)",
				cmp_crc[0], cmp_crc[1]);
		ret = 0;
	} else {
		log_submit(eMessageTypeNote, "Different CRC values (.crc file:0x%X "
				"0x%X, computed:0x%X 0x%X)", cmp_crc[0], cmp_crc[1],
				rcv_crc[0], rcv_crc[1]);
		ret = 1;
	}

	return ret;
}

/**
 * @brief Reboot/switch the microcontroller from BSL to applicative firmware
 *
 * Send a reboot frame in order to reboot firmware of a microcontroller
 * (currently in BSL mode) to applicative mode
 *
 * If reboot sending/receiving fails, requests applicative wakeup
 * 
 * * Send: CAN_ID_BSL_ACTION (reboot to app mode)
 * * Receive: CAN_ID_WAKEUP_FRAME
 *
 * @param[in] card type of microcontroller (APFB, HIL or LLD)
 * @param[in] pnp plug and play integer, depending the microcontroller instance
 *
 * @return 0 if succeeded, -1 else
 */
static int bsl_launch_app(const e_canCard card, const unsigned int pnp)
{
	int ret = 0;
	int expected_id = wrapper_can_calculate_frame_id(card, pnp,
			CAN_ID_WAKEUP_FRAME);
	s_canFrame rcv_frame, snd_frame = { .id = 0, .dlc = 0, .data = { 0 } };

	snd_frame.id = CAN_ID_BSL_ACTION + pnp;
	snd_frame.dlc = 3;
	snd_frame.data[0] = 8;
	snd_frame.data[1] = 0;
	snd_frame.data[2] = 0;

	ret = send_frame_and_wait(snd_frame, &(rcv_frame), expected_id,
			CAN_REBOOT_TIMEOUT);
	if (ret == 0)
		return ret;

	/* Wakeup frame not received, so we send an new APP wakeup */
	log_submit(eMessageTypeWarning, "APP wakeup frame 0x%X from card %u, pnp "
			"%u not received, request a new APP wakeup", expected_id, card,
			pnp);

	/* Request a wakeup for APP */
	return app_request_wakeup(card, pnp, CAN_MAX_RETRIES);
}

/**
 * @brief Switch the firmware of the slaveboard linked to the microcontrollers
 *
 * Send a word to the slaveboard linked to the microcontrollers in order to
 * restart its in this opposite firmware mode (applicative to factory OR
 * factory to applicative). A sleep of X seconds is performed between halt and
 * start of Openpowerlink stack (time for the Slaveboard to reboot)
 *
 * @param[in] hnnode hardware node structure
 * @param[in] woplk_ctxt OpenPowerLink wrapper context (cycle, time
 * sleeping, state)
 * @param[in] mnobd_path path of the 'mnobd.cdc' file to use
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_can_switch_factory(struct hardware_node *hnnode,
		struct wrapper_context *woplk_ctxt, char *mnobd_path) {
	int ret;
	tOplkError                  oplkRet;
	const NodeInfoArray*		found_nodes = NULL;

	/* Check if OpenPowerLink wrapper is enabled */
	if (woplk_ctxt->is_on == false) {
		log_submit(eMessageTypeError, "OpenPowerLink wrapper must be enabled "
				"in order to switch node %d (%s) in factory node",
				hnnode->oplk_idnode, hnnode->name);
		return -1;
	}

	/*
	 * Slaveboard production firmware doesn't recept new firmware by default.
	 * So we send a command word (by PDO) to enable factory mode (that
	 * accepts new firmware)
	 * Initialization command word
	 * It is possible that slaveboard doesn't support upload protocol and
	 * doesn't have factory firmware (like before Fwupdater). So when 10
	 * seconds happened, we know slaveboard cannot be updated
	 */
	ret = wrapper_send_PDO_command_word_time(&wrapper_PDO_ctxt,
			BSL_CMD_INIT, 10);
	if (ret) {
		if (ret == -3)
			log_submit(eMessageTypeError, "No response in firmware switch "
					"mode command to node %d (%s). Maybe it doesn't "
					"support upload protocol and doesn't have "
					"factory firmware", hnnode->oplk_idnode,
					hnnode->name);
		return ret;
	}
	if (wrapper_PDO_ctxt.error) {
		log_submit(eMessageTypeError, "Error in PDO operations");
		return -1;
	}

	/* Switch to factory command word */
	if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt,
				BSL_CMD_SWITCH_TO_FACTORY) == -2) {
		return -2;
	}
	if (wrapper_PDO_ctxt.error) {
		log_submit(eMessageTypeError, "Error in PDO operations");
		return -1;
	}

	/* Halt OpenPowerLink bus */
	if ((oplkRet = shutdownPowerlink()) != kErrorOk) {
		log_submit(eMessageTypeError, "Error in shutdown of OpenPowerLink "
				"wrapper : \"%s\" (0x%04x)",
				debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
		return -1;
	}
	/* The wrapper is now in state "off" */
	woplk_ctxt->is_on = false;

	/*
	 * Wait restart Slaveboard firmware to factory mode
	 * Important: sleep() can be used for a thread, it
	 * doesn't stop all threads but only the calling one
	 */
	log_submit(eMessageTypeNote, "Wait %d seconds for factory firmware "
			"starting of node %d (\"%s\")", woplk_ctxt->sleep,
			hnnode->oplk_idnode, hnnode->name);

	if (sleep(woplk_ctxt->sleep) > 0) {
		log_submit(eMessageTypeError, "Interrupted sleeping");
		return -2;
	}

	/* Initialize OpenPowerLink wrapper */
	ret = wrapper_can_initialize_oplk(mnobd_path, woplk_ctxt->cycle);
	if (ret != 0) {
		log_submit(eMessageTypeError, ret == -2 ? "Initialization aborted by "
				"user" : "OPLK wrapper (for CAN node) cannot be loaded");
		return ret;
	}
	/* The wrapper is now in state "on" */
	woplk_ctxt->is_on = true;

	/*
	 * This function will return kErrorOk only if all nodes in the cdc where
	 * found and synced. If one node was synced but lead to an error, its
	 * state will be set accordingly
	 */
	if (wOplkIdentifyNodes(&found_nodes) != kErrorOk || found_nodes == NULL) {
		log_submit(eMessageTypeError, "Cannot identify OpenPowerlink nodes");
		return -1;
	}

	/* Now, node is in factory mode */
	hnnode->is_factory_mode = true;

	return 0;
}

/**
 * @brief Update the firmware of all microcontrollers
 *
 * Detects microcontroller type of node (APFB, HIL or LLD) and sends
 * firmwares to boards of the specified microcontroller type
 *
 * Note: microcontrollers boards are fixed numbering. So we use
 * static arrays to send firmware to the right microcontroller
 *
 * @param[in] hnnode hardware node structure
 * @param[in] woplk_ctxt OpenPowerLink wrapper context (cycle, time
 * sleeping, state)
 * @param[in] bindir directory where to find firmware binaries
 * @param[in] oplknode OpenPowerLink node structure, used to get
 * its .cdc file path
 * and to detect its mode and (if needed) switch in factory mode
 * @param[in] launch_oplk_task used to enable OpenPowerLink launching ("false"
 * if previous execution of this function has skipped OpenPowerLink halt)
 * @param[in] post_task used to prevent return to production mode of slaveboard
 * and/orcOpenPowerLink wrapper disabling
 * @param[in,out] prog_trk pointer to a GUI flashing progression structure
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_can_send_firmwares(struct hardware_node *hnnode,
		struct wrapper_context *woplk_ctxt, char *bindir,
		struct hardware_node *oplknode, bool launch_oplk_task,
		eCanWrapperTask post_task, progress_tracker_t *prog_trk) {
	int i, ret, cmp_version;
	char crc_path[PATH_MAX], bin_path[PATH_MAX], mnobd_path[PATH_MAX];
	tOplkError oplkRet;
	const NodeInfoArray* found_nodes = NULL;
	struct can_mc_metadata {
		int max_mc_nodes;
		unsigned int const *id_mc_array;
		e_canCard card_type;
	} mc_meta;
	/* 21 is to comply with Stago versions */
	char version[21];

	/* Loading of slaveboard .cdc */
	/* Concatenate firmware name with binaries' directory */
	if (snprintf(mnobd_path, PATH_MAX, "%s/%s", bindir, oplknode->sb_binary) >= PATH_MAX) {
		log_submit(eMessageTypeError, "Path too long");
		return -1;
	}

	/* Check existence and regularity of .cdc file */
	if ((process_file_is_regular(mnobd_path)) == -1) {
		log_submit_perror(eMessageTypeError, ".cdc file %s unusable", mnobd_path);
		return -1;
	}

	if (launch_oplk_task) {
		/* Initialize OpenPowerLink wrapper */
		ret = wrapper_can_initialize_oplk(mnobd_path, woplk_ctxt->cycle);
		if (ret != 0) {
			log_submit(eMessageTypeError, ret == -2 ? "Initialization aborted "
					"by user" : "OPLK wrapper (for CAN node) cannot be loaded");
			return ret;
		}
		/* The wrapper is now in state "on" */
		woplk_ctxt->is_on = true;

		/*
		 * This function will return kErrorOk only if all nodes in the cdc where
		 * found and synced. If one node was synced but lead to an error, its state
		 * will be set accordingly
		 */
		if (wOplkIdentifyNodes(&found_nodes) != kErrorOk || found_nodes == NULL) {
			log_submit(eMessageTypeError, "Cannot identify OpenPowerlink nodes");
			return -1;
		}
	}

	/* If node is not in factory mode */
	if (!oplknode->is_factory_mode) {
		log_submit(eMessageTypeNote, "Switch node %d (%s) to factory mode",
				oplknode->oplk_idnode, oplknode->name);
		ret = wrapper_can_switch_factory(oplknode, woplk_ctxt, mnobd_path);
		if (ret)
			return ret;
	}

	/*
	 * Concatenate firmware name with binaries' directory
	 * Note: each microcontroller has only one binary. So
	 * we suppose that binary is at position zero
	 */
	if (snprintf(bin_path, PATH_MAX, "%s/%s", bindir, hnnode->files[0].base_name) >= PATH_MAX) {
		log_submit(eMessageTypeError, "Path too long");
		return -1;
	}
	/*
	 * Concatenate CRC name with binaries' directory
	 * Note: CRC file is obtained by context_add_hardware_node_file()
	 * function. It is added at position one
	 */
	if (snprintf(crc_path, PATH_MAX, "%s/%s", bindir, hnnode->files[1].base_name) >= PATH_MAX) {
		log_submit(eMessageTypeError, "Path too long");
		return -1;
	}

	if (strequ(hnnode->name, "APFB")) {
		mc_meta.max_mc_nodes = NB_APFB_NODES;
		mc_meta.id_mc_array = APFB_CARD_IDS;
		mc_meta.card_type = e_canCardAPFB;
	} else if (strequ(hnnode->name, "HIL")) {
		mc_meta.max_mc_nodes = NB_HIL_NODES;
		mc_meta.id_mc_array = HIL_CARD_IDS;
		mc_meta.card_type = e_canCardHIL;
	} else if (strequ(hnnode->name, "LLDCAN")) {
		mc_meta.max_mc_nodes = NB_LLD_NODES;
		mc_meta.id_mc_array = LLD_CARD_IDS;
		mc_meta.card_type = e_canCardLLD;
	} else {
		log_submit(eMessageTypeError, "Unknown microcontroller "
				"(%s)", hnnode->name);
		return -1;
	}

	if (prog_trk != NULL){
		prog_trk->iteration_count = mc_meta.max_mc_nodes;
	}

	log_submit(eMessageTypeNote, "Send file %s (with its .crc file) to "
			"microcontroller nodes \"%s\"", bin_path,
			mcu_to_string(mc_meta.card_type));

	/* For each APFB/HIL/LLD card */
	for (i = 0 ; i < mc_meta.max_mc_nodes ; i++) {
		/* If subnode is not to flash */
		if (hnnode->mc_subnode[i].to_update == false)
			continue;

		log_submit(eMessageTypeNote, "Update microcontroller node: [name:%s "
				"pnp:%d]", mcu_to_string(mc_meta.card_type), mc_meta.id_mc_array[i]);

		/* If subnode is not in factory mode */
		if (hnnode->mc_subnode[i].is_factory_mode == false) {

			log_submit(eMessageTypeNote, "Switch the microcontroller to BSL "
					"mode: [name:%s pnp:%d]", hnnode->name,
					mc_meta.id_mc_array[i]);

			/* In user/app mode, enter in BSL mode */
			if (app_reboot_in_bsl(mc_meta.card_type, mc_meta.id_mc_array[i]) != 0) {
				log_submit(eMessageTypeError, "Failed to enter in BSL mode for "
						"microcontroller [name:%s pnp:%d]", hnnode->name,
						mc_meta.id_mc_array[i]);
				return -1;
			}
			/* Subnode is now in factory mode */
			hnnode->mc_subnode[i].is_factory_mode = true;
		}

		ret = wrapper_can_flash_app(mc_meta.card_type, mc_meta.id_mc_array[i],
				bin_path, crc_path, prog_trk);

		/* For future updates */
		hnnode->mc_subnode[i].is_factory_mode = false;

		if (ret) {
			if (ret == -1)
				log_submit(eMessageTypeError, "Error in flashing the microcontroller: "
						"[name:%s pnp:%d] (%d)", mcu_to_string(mc_meta.card_type),
						mc_meta.id_mc_array[i], ret);
			return ret;
		}

		/* Get version of the flashed subnode */
		ret = app_get_version(mc_meta.card_type, mc_meta.id_mc_array[i], version);
		if (ret != 0) {
			log_submit(eMessageTypeError, "Couldn't get version of microcontroller: "
					"[name:%s pnp:%d]", mcu_to_string(mc_meta.card_type),
					mc_meta.id_mc_array[i]);
			return -1;
		}

		/*
		 * Compare software version between XML version
		 * and got version (in OpenPowerlink node)
		 */
		cmp_version = context_compare_version(hnnode->version, version);
		switch(cmp_version) {
			/*
			 * If XML defined version of node is equals to
			 * got version, we have correctly flashed the
			 * subnode
			 */
			case 0:
				log_submit(eMessageTypeNote, "Found subnode: [name:%s pnp:%d "
						"version:%s] -> no need to flash (same version)",
						hnnode->name, mc_meta.id_mc_array[i], version);
				break;
			case -1:
				log_submit(eMessageTypeError, "Bad software version format "
						"(%s) for subnode [name:%s pnp:%d]", version,
						hnnode->name, mc_meta.id_mc_array[i]);
				return -1;
				break;
			/* Error, subnode incorrectly flashed */
			default:
				log_submit(eMessageTypeNote, "Found subnode: [name:%s pnp:%d "
						"version:%s] -> no up-to-date",
						hnnode->name, mc_meta.id_mc_array[i], version);
				return -1;
				break;
		}
		if (prog_trk != NULL) {
			prog_trk->iteration_index += 1;
		}
	}

	/*
	 * If we have to stop OpenPowerLink wrapper or also
	 * to switch back slaveboard to production mode
	 */
	if ((post_task == eCanWrapperTaskShutdown) ||
			(post_task == eCanWrapperTaskSwitchProduction)) {

		/* Come back the microcontroller's firmware to production mode */
		if (post_task == eCanWrapperTaskSwitchProduction) {
			/* Come back to user mode command word */
			if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt,
						BSL_CMD_SWITCH_TO_USER) == -2)
				return -2;

			if (wrapper_PDO_ctxt.error) {
				log_submit(eMessageTypeError, "Error in PDO operations");
				return -1;
			}
			/* Now, node is in production mode */
			oplknode->is_factory_mode = false;
		}

		if ((oplkRet = shutdownPowerlink()) != kErrorOk) {
			log_submit(eMessageTypeError, "Error in shutdown of "
					"OpenPowerLink wrapper : \"%s\" (0x%04x)",
					debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
			return -1;
		}
		/* The wrapper is now in state "off" */
		woplk_ctxt->is_on = false;
	}

	return 0;
}
