#include <assert.h>
#include "include/fw_wrapper.h"
#include "include/fw_wrapper_can.h"
#include "include/fw_wrapper_oplk.h"
#include "include/fw_wrapper_pcie.h"

/* For log_level_e type */
#include "common/logMgmt.h"
/* Define wrappers logging level to "warning" */
log_level_e log_level = LOG_LEVEL_WARNING;

/**
 * @brief Wait something (a mutex unlock)
 *
 * Wait (lock a mutex) something. pthread_cond_wait() may
 * be cancelled by SIGTERM, so we check if it has been by
 * this way
 *
 * @param[in] wcond pthread condition
 * @param[in] wmutex pthread mutex
 *
 * @return 0 if succeeded, -1 if fails, -2 if user has cancelled
 */
int wrapper_wait(pthread_cond_t *wcond, pthread_mutex_t *wmutex)
{
	if (pthread_cond_wait(wcond, wmutex) != 0) {
		log_submit(eMessageTypeWarning, "Thread waiting error");
		return -1;
	}

	if (fTermSignalReceived_l == true) {
		log_submit(eMessageTypeWarning, "Waiting cancelled by user "
				"(terminating signal sent)");
		return -2;
	}
	return 0;
}

/**
 * @brief Like function wrapper_wait(), but with timeout
 *
 * Wait (lock a mutex) something with a expiration time.
 * pthread_cond_timedwait() may be cancelled by SIGTERM, so
 * we check if it has been by this way
 *
 * @param[in] wcond pthread condition
 * @param[in] wmutex pthread mutex
 * @param[in] time timeout (in seconds)
 *
 * @return Return code:
 * *  0: mutex freed without timer expiration
 * * -1: error
 * * -2: cancel wait (typically CTRL + C)
 * * -3: timer expired
 */
int wrapper_wait_time(pthread_cond_t *wcond, pthread_mutex_t *wmutex, int time)
{
	int ret;
	struct timespec wait_time;
	struct timeval now;

	/* Get current date */
	gettimeofday(&now, NULL);
	/* Set 15 seconds waiting */
	wait_time.tv_sec = now.tv_sec + time;
	wait_time.tv_nsec = 0;

	ret = pthread_cond_timedwait(wcond, wmutex, &wait_time);
	if (ret) {
		if (ret == ETIMEDOUT) {
			log_submit(eMessageTypeWarning, "Wait timed out");
			return -3;
		} else {
			log_submit(eMessageTypeWarning, "Thread waiting error");
			return -1;
		}
	}

	/* We go here if timer did not expire (and no error) */
	if (fTermSignalReceived_l == true) {
		log_submit(eMessageTypeWarning, "Waiting cancelled by user "
				"(terminating signal sent)");
		return -2;
	}
	return 0;
}

/**
 * @brief Scan boards in order to verify their update success
 *
 * Used after boards update, this function checks if these
 * boards are correclty flashed (same versions).
 *
 * @param[in] hnlist hardware nodes list
 * @param[in] wclist wrapper contexts list
 * @param[in] bindir directory where to find firmwares to send
 *
 * @return Return code:
 * *  0: all boards successfully flashed
 * * -1: error
 * * -2: cancel wait (typically CTRL + C)
 */
int wrapper_verify(struct hardware_node_list *hnlist, struct
		wrapper_context_list *wclist, char *bindir)
{
	int			i, ret = -1;
	char		mnobd_path[PATH_MAX];
	tOplkError	oplkRet;
	tPcieWrapperError pcieRet;

	assert(wclist->wrapper[0].instance == eWrapperOPLK);

	sleep(10);

	/* For each wrapper */
	for (i = 0 ; i < wclist->count ; i++) {
		if (wclist->wrapper[i].instance == eWrapperOPLK) {
			/*
			 * No OpenPowerlink node has been updated
			 * (and microcontroller neither)
			 */
			if (hnlist->count_by_wrapper[eWrapperOPLK-1] == 0)
				continue;

			/* Concatenate firmware name with binaries' directory */
			if (snprintf(mnobd_path, PATH_MAX, "%s/%s", bindir,
						wclist->wrapper[i].binary) >= PATH_MAX) {
				log_submit(eMessageTypeError, "Path too long");
				return -1;
			}
			/* Check existence and regularity of mnobd.cdc file */
			if ((process_file_is_regular(mnobd_path)) == -1) {
				log_submit_perror(eMessageTypeError, "mnobd.cdc file "
						"\"%s\" unusable", mnobd_path);
				return -1;
			}
			if (wrapper_oplk_initialize(mnobd_path, wclist->wrapper[i].cycle) == -1) {
				log_submit(eMessageTypeError, "OPLK wrapper cannot be loaded");
				return -1;
			}
			if (wrapper_oplk_scan_network(hnlist, true, NULL) == -1) {
				log_submit(eMessageTypeError, "Error or mismatch in detected "
						"OpenPowerlink nodes");
				return -1;
			}
			/*
			 * Stop OpenPowerLink wrapper, because we only
			 * wanted to scan network and detect nodes
			 */
			if ((oplkRet = shutdownPowerlink()) != kErrorOk) {
				log_submit(eMessageTypeError, "Error in shutdown of "
						"OpenPowerLink wrapper : \"%s\" (0x%04x)",
						debugstr_getOplkWrapperRetValStr(oplkRet),
						oplkRet);
				return -1;
			}
			/* The wrapper is now in state "off" */
			wclist->wrapper[i].is_on = false;
		} else if (wclist->wrapper[i].instance == eWrapperPCIE) {
			/* No PCI Express node to update */
			if (hnlist->count_by_wrapper[eWrapperPCIE-1] == 0)
				continue;

			/*
			 * Initialize PCI Express wrapper
			 * Note: no file to check (contrary to OpenPowerLink
			 * wrapper) because PCIe wrapper have no file
			 */
			ret = wrapper_pcie_initialize(wclist->wrapper[i].cycle);
			if (ret) {
				log_submit(eMessageTypeError, (ret == -2 ? "PCIE wrapper "
							"initialization aborted" : "PCIE wrapper "
							"cannot be loaded"));
				return ret;
			}
			/* The wrapper is set as "on" */
			wclist->wrapper[i].is_on = true;

			if (wrapper_pcie_scan_bus(hnlist, true, NULL) == -1) {
				log_submit(eMessageTypeError, "Error or mismatch in "
						"detected \"mainboard\" node");
				return -1;
			}

			/* Stop PCIe wrapper */
			if ((pcieRet = wPcieCloseBus()) != pcieNoError) {
				log_submit(eMessageTypeError, "Error in shutdown of "
						"PCI Express wrapper : 0x%04x", pcieRet);
				return -1;
			}
			/* The wrapper is now in state "off" */
			wclist->wrapper[i].is_on = false;
		}
	}

	return 0;
}

/**
 * @brief Initialize wrappers and check if all needed boards are presents
 *
 * This function tests that wrappers can start and if every board to update
 * is present.
 *
 * @param[in] hnlist hardware nodes list
 * @param[in] wclist wrapper contexts list
 * @param[in] bindir directory where to find firmwares to send
 * @param[in,out] prog_trk pointer to a GUI flashing progression structure
 *
 * @return Return code:
 * *  0: all boards successfully flashed
 * * -1: error
 * * -2: cancel wait (typically CTRL + C)
 */
int wrapper_initialize(struct hardware_node_list *hnlist, struct
		wrapper_context_list *wclist, char *bindir,
		progress_tracker_t *prog_trk)
{
	int			i, ret = -1;
	char		mnobd_path[PATH_MAX];
	tOplkError	oplkRet;
	tPcieWrapperError pcieRet;
	bool		mc_oplk_node_enabled = true;

	mutex = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
	condStackOff = (pthread_cond_t) PTHREAD_COND_INITIALIZER;
	//log_level = LOG_LEVEL_DEBUG;

	/* For each wrapper */
	for (i = 0 ; i < wclist->count ; i++) {
		if (wclist->wrapper[i].instance == eWrapperOPLK) {
			/* Neither OpenPowerlink node nor microcontroller to update */
			if (hnlist->count_by_wrapper[eWrapperOPLK-1] == 0)
				continue;

			/* Concatenate firmware name with binaries' directory */
			if (snprintf(mnobd_path, PATH_MAX, "%s/%s", bindir, wclist->wrapper[i].binary) >= PATH_MAX) {
				log_submit(eMessageTypeError, "Path too long");
				return -1;
			}
			/* Check existence and regularity of .cdc file */
			if ((process_file_is_regular(mnobd_path)) == -1) {
				log_submit_perror(eMessageTypeError, ".cdc file \"%s\" "
						"unusable", mnobd_path);
				return -1;
			}

			ret = wrapper_oplk_initialize(mnobd_path, wclist->wrapper[i].cycle);
			if (ret) {
				log_submit(eMessageTypeError, (ret == -2 ? "OPLK wrapper "
							"initialization aborted" : "OPLK wrapper "
							"cannot be loaded"));
				return ret;
			}
			/* The wrapper is now in state "on" */
			wclist->wrapper[i].is_on = true;

			// mc_oplk_node exists if there are any microcontrollers (MCUs) to be updated in context.xml
			// MCUs must be updated through Slaveboard T
			// Slaveboard T must be enabled (to_update = true)
			if (hnlist->mc_oplk_node && !hnlist->mc_oplk_node->to_update) {
				// Put temporarily to true
				hnlist->mc_oplk_node->to_update = true;
				// Remember to disable it back later
				mc_oplk_node_enabled = false;
				if (prog_trk != NULL) {
					prog_trk->ignore_trans_sb = true;
				}
			}

			if (wrapper_oplk_scan_network(hnlist, false, prog_trk) == -1) {
				log_submit(eMessageTypeError, "Error or mismatch in detected "
						"OpenPowerlink nodes");
				return -1;
			}

			if (hnlist->mc_oplk_node && mc_oplk_node_enabled == false)
				hnlist->mc_oplk_node->to_update = false;

			/*
			 * Stop OpenPowerLink wrapper, because we only
			 * wanted to scan network and detect nodes
			 */
			if ((oplkRet = shutdownPowerlink()) != kErrorOk) {
				log_submit(eMessageTypeError, "Error in shutdown of "
						"OpenPowerLink wrapper : \"%s\" (0x%04x)",
						debugstr_getOplkWrapperRetValStr(oplkRet),
						oplkRet);
				return -1;
			}
			/* The wrapper is now in state "off" */
			wclist->wrapper[i].is_on = false;

			/* Now, we will check microcontrollers */

			/* If we don't have microcontroller node to flash */
			if (!hnlist->mc_oplk_node)
				continue;

			/* Loading of slaveboard .cdc */
			/* Reinitialize buffer */
			memset(mnobd_path, '\0', sizeof(char) * PATH_MAX);
			/* Concatenate 'mnobd.cdc' path with binaries' directory */
			if (snprintf(mnobd_path, PATH_MAX, "%s/%s", bindir,
						hnlist->mc_oplk_node->sb_binary) >= PATH_MAX) {
				log_submit(eMessageTypeError, "Path too long");
				return -1;
			}
			/*
			 * Initialize OpenPowerLink wrapper for the
			 * slaveboard connected to microcontrollers
			 */
			if (wrapper_can_initialize_oplk(mnobd_path, wclist->wrapper[i].cycle) == -1) {
				log_submit(eMessageTypeError, "OPLK wrapper cannot be loaded");
				return -1;
			}
			/* The wrapper is now in state "on" */
			wclist->wrapper[i].is_on = true;

			/* If node is not in factory mode */
			if (!hnlist->mc_oplk_node->is_factory_mode) {
				log_submit(eMessageTypeNote, "Switch node %d (%s) to factory "
						"mode", hnlist->mc_oplk_node->oplk_idnode,
						hnlist->mc_oplk_node->name);
				ret = wrapper_can_switch_factory(hnlist->mc_oplk_node,
						&(wclist->wrapper[i]), mnobd_path);
				if (ret)
					return ret;
			}

			ret = wrapper_can_scan_bus(hnlist, prog_trk);
			if (ret) {
				log_submit(eMessageTypeError, "Error or mismatch in detected "
						"CAN/microcontrollers nodes");
				return -1;
			}

			/* Stop OpenPowerLink wrapper */
			if ((oplkRet = shutdownPowerlink()) != kErrorOk) {
				log_submit(eMessageTypeError, "Error in shutdown of "
						"OpenPowerLink wrapper : \"%s\" (0x%04x)",
						debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
				return -1;
			}
			/* The wrapper is now in state "off" */
			wclist->wrapper[i].is_on = false;
		} else if (wclist->wrapper[i].instance == eWrapperPCIE) {
			/* No PCI Express node to update */
			if (hnlist->count_by_wrapper[eWrapperPCIE-1] == 0)
				continue;

			/*
			 * Initialize PCI Express wrapper
			 * Note: no file to check (contrary to OpenPowerLink
			 * wrapper) because PCIe wrapper have no file
			 */
			ret = wrapper_pcie_initialize(wclist->wrapper[i].cycle);
			if (ret) {
				log_submit(eMessageTypeError, (ret == -2 ? "PCIE wrapper "
							"initialization aborted" : "PCIE wrapper cannot be "
							"loaded"));
				return ret;
			}
			/* The wrapper is set as "on" */
			wclist->wrapper[i].is_on = true;

			if (wrapper_pcie_scan_bus(hnlist, false, prog_trk) == -1) {
				log_submit(eMessageTypeError, "Error or mismatch in detected "
						"\"mainboard\" node");
				return -1;
			}

			/* Stop PCIe wrapper */
			if ((pcieRet = wPcieCloseBus()) != pcieNoError) {
				log_submit(eMessageTypeError, "Error in shutdown of PCI "
						"Express wrapper : 0x%04x", pcieRet);
				return -1;
			}
			/* The wrapper is now in state "off" */
			wclist->wrapper[i].is_on = false;
		}
	}

	return 0;
}

/**
 * @brief Stop wrappers and clean mutex
 *
 * Shutdown all wrappers and free mutex used for synchronization
 *
 * @param[in] wclist wrapper contexts list
 */
void wrapper_free(struct wrapper_context_list *wclist)
{
	int i;
	tOplkError oplkRet;
	tPcieWrapperError pcieRet;

	for (i = 0 ; i < wclist->count ; i++) {
		/* If wrapper is not active, useless to shutdown it */
		if (!wclist->wrapper[i].is_on)
			continue;

		/* Close OpenPowerLink wrapper */
		if (wclist->wrapper[i].instance == eWrapperOPLK) {
			if ((oplkRet = shutdownPowerlink()) != kErrorOk)
				log_submit(eMessageTypeError, "Error in shutdown of "
						"OpenPowerLink wrapper : \"%s\" (0x%04x)",
						debugstr_getOplkWrapperRetValStr(oplkRet),
						oplkRet);
		/* Close PCI Express wrapper */
		} else if (wclist->wrapper[i].instance == eWrapperPCIE) {
			if ((pcieRet = wPcieCloseBus()) != pcieNoError)
				log_submit(eMessageTypeError, "Error in shutdown of PCI "
						"Express wrapper. Code:%d", pcieRet);
		}
	}

	/*
	 * Important: we release mutex only after wrappers closing. If we do it
	 * before, we may have deadlock, especially with PCIe wrapper. Technically,
	 * wrapper use FwUpdater function to manage events
	 * (pcieWrapperEventCallback()). This function calls pthread_cond_signal(),
	 * but if mutex is released before this function will cause deadlock
	 * because mutex doesn't exist anymore
	 */
	pthread_mutex_destroy(&mutex);
	pthread_cond_destroy(&condStackOff);
}

/**
 * @brief Extract firmware binairies and flashes boards
 *
 * Flash all hardware nodes respecting their index (defined in XML file)
 * Decompress/delete also node's binaries (respectively before and after
 * flashing)
 *
 * @param[in] hnlist hardware nodes list
 * @param[in] wclist wrapper contexts list
 * @param[in] bindir directory where to find firmwares to send
 * @param[in,out] prog_trk pointer to a GUI flashing progression structure
 *
 * @return 0 if succeeded, -1 if fails, -2 if user has cancelled
 */
int wrapper_send_firmwares(struct hardware_node_list *hnlist,
		struct wrapper_context_list *wclist, char *bindir,
		progress_tracker_t *prog_trk)
{
	int i, ret = 0;
	struct wrapper_context *wc_oplk = NULL, *wc_pcie = NULL;
	char archive[PATH_MAX];
	/*
	 * Performance tip: we use a binary mask with the n th bit to 1 if the node
	 * "n" is a microcontroller to flash. This mask allows to prevent to halt
	 * OpenPowerLink wrapper or switch back slaveboard firmware to user mode
	 * when there are still microcontroller nodes to flash
	 *
	 * So, we shutdown wrapper only if next node to flash is not a
	 * microcontroller. And we switch back to user mod only there is no more
	 * microcontroller node to flash
	 */
	int can_mask = 0;
	/*
	 * Used to keep trace of OpenPowerLink wrapper state
	 * for CAN between each microcontroller node
	 */
	bool can_launch_oplk = true;

	/* Create binary mask regarding microcontrollers to flash */
	for (i = 0 ; i < hnlist->count ; i++) {
		if (hnlist->node[i].to_update && hnlist->node[i].bustype == eBusTypeCAN)
			can_mask |= (1 << i);
	}

	/* Define pointers to wrappers structures */
	for (i = 0 ; i < wclist->count ; i++) {
		if (wclist->wrapper[i].instance == eWrapperOPLK)
			wc_oplk = &(wclist->wrapper[i]);
		else if (wclist->wrapper[i].instance == eWrapperPCIE)
			wc_pcie = &(wclist->wrapper[i]);
	}

	/*
	 * Flash each node
	 *
	 * Reminder: when a node is added to nodes list (context_add_hardware_node()
	 * function), its position is according to index defined in XML. So, a
	 * simple 'for' loop guarantees good order of flashing operation
	 */
	for (i = 0; i < hnlist->count ; i++) {

		/* If flashing of that node has been disabled */
		if (!hnlist->node[i].to_update)
			continue;

		if (prog_trk != NULL) {
			prog_trk->package = hnlist->node[i].archive;
			prog_trk->iteration_index = 0;
		}
		/* Initialize areas */
		memset(archive, '\0', sizeof(archive));

		/* Get path to node archive */
		if (snprintf(archive, PATH_MAX, "%s/%s", bindir, hnlist->node[i].archive) >= PATH_MAX) {
			log_submit(eMessageTypeError, "Path too long");
			ret = -1;
			goto err;
		}

		/* Extract the binaries of the node */
		if (zip_extract_binaries(archive, bindir, &(hnlist->node[i])) == -1) {
			ret = -1;
			goto err;
		}

		/* PCIE: If the node is a Mainboard and does use PCI bus */
		if (hnlist->node[i].type == eHardwareNodeTypeMainboard
			&& hnlist->node[i].bustype == eBusTypePCI) {
			if ((ret = wrapper_pcie_send_firmwares(&hnlist->node[i], wc_pcie,
							bindir, prog_trk)) == -1)
				log_submit(eMessageTypeError, "Couldn't send firmware(s) "
						"to PCI Express node");
			if (prog_trk != NULL) {
				if (ret == 0) {
					notify_success(prog_trk);
				}
				else {
					notify_failure(prog_trk);
				}
			}
			if (ret == -1 || ret == -2)
				goto err_delete;
		}

		/* OPLK: OpenPowerLink node */
		else if (hnlist->node[i].bustype == eBusTypeEPL) {
			ret = wrapper_oplk_send_firmwares(&hnlist->node[i], wc_oplk,
					bindir, prog_trk);
			if (prog_trk != NULL) {
				if (ret == 0)
					notify_success(prog_trk);
				else
					notify_failure(prog_trk);
			}
			if (ret) {
				/* If there is error or SDO response wait has expired */
				if (ret == -1 || ret == -3)
					log_submit(eMessageTypeError, "Couldn't send firmware(s) "
							"to OpenPowerlink node");

				goto err_delete;
			}
		}

		/* CAN: CAN node (microcontroller) */
		else if (hnlist->node[i].bustype == eBusTypeCAN) {
			/*
			 * Send firmware to microcontroller node using .cdc file of
			 * OpenPowerlink node. That OpenPowerlink is the one used
			 * to reach microcontrollers
			 */

			/*
			 * If there is no more node to the hardware list
			 * Note: "i+1" is to convert actual position to
			 * human counting (1, 2, 3...)
			 */
			if (i+1 == hnlist->count) {
				/*
				 * If the slaveboard is not to flash and was already
				 * in factory mode, keep it in that mode
				 */
				if ((hnlist->mc_oplk_node->to_update == false) &&
						(hnlist->mc_oplk_node->was_factory_mode == true)) {
					log_submit(eMessageTypeNote, "Will keep the slaveboard "
							"connected to microcontroller nodes in factory "
							"mode, because not included in update process");
					ret = wrapper_can_send_firmwares(&hnlist->node[i], wc_oplk,
							bindir, hnlist->mc_oplk_node, can_launch_oplk,
							eCanWrapperTaskShutdown, prog_trk);
				/*
				 * The slaveboard was flashed or was already
				 * in production mode at FwUpdater starting
				 */
				} else {
					ret = wrapper_can_send_firmwares(&hnlist->node[i], wc_oplk,
							bindir, hnlist->mc_oplk_node, can_launch_oplk,
							eCanWrapperTaskSwitchProduction, prog_trk);
				}
			/*
			 * If the next node is a microcontroller to flash
			 * Note: "i+1" is to match next position
			 */
			} else if (((can_mask >> (i+1)) & 0x1) == 1) {
				ret = wrapper_can_send_firmwares(&hnlist->node[i], wc_oplk,
						bindir, hnlist->mc_oplk_node, can_launch_oplk,
						eCanWrapperTaskNothing, prog_trk);
				can_launch_oplk = false;
				log_submit(eMessageTypeNote, "Will keep OpenpowerLink enabled "
						"for the next microcontroller node");
			/* If there are later another microcontrollers nodes to flash */
			} else if ((can_mask >> (i+1)) > 0) {
				ret = wrapper_can_send_firmwares(&hnlist->node[i], wc_oplk,
						bindir, hnlist->mc_oplk_node, can_launch_oplk,
						eCanWrapperTaskShutdown, prog_trk);
				can_launch_oplk = true;
				log_submit(eMessageTypeNote, "Will keep the slaveboard "
						"connected to microcontroller nodes in factory "
						"mode, for next update(s)");
			/* If there is no more microcontroller node to flash */
			} else {
				/*
				 * If the slaveboard is not to flash and was already
				 * in factory mode, keep it in that mode
				 *
				 * OR
				 *
				 * If the slaveboard has to be flashed BUT after the last
				 * microcontroller (if we switch the slaveboard mode, we may
				 * have an error because the slaveboard hasn't always a
				 * production firmware)
				 */
				if (((hnlist->mc_oplk_node->to_update == false) &&
						(hnlist->mc_oplk_node->was_factory_mode == true)) ||
						((hnlist->mc_oplk_node->to_update == true) &&
						(hnlist->mc_oplk_node > &(hnlist->node[i])))) {
					log_submit(eMessageTypeNote, "Will keep the slaveboard "
							"connected to microcontroller nodes in factory "
							"mode, because not included in update process");
					ret = wrapper_can_send_firmwares(&hnlist->node[i], wc_oplk,
							bindir, hnlist->mc_oplk_node, can_launch_oplk,
							eCanWrapperTaskShutdown, prog_trk);
				} else {
					log_submit(eMessageTypeNote, "Will switch back the "
							"slaveboard connected to microcontroller "
							"nodes to production mode");
					ret = wrapper_can_send_firmwares(&hnlist->node[i], wc_oplk,
							bindir, hnlist->mc_oplk_node, can_launch_oplk,
							eCanWrapperTaskSwitchProduction, prog_trk);
				}
			}

			if (prog_trk != NULL) {
				if (ret == 0)
					notify_success(prog_trk);
				else
					notify_failure(prog_trk);
			}
			if (ret) {
				/* If there is error or SDO response wait has expired */
				if (ret == -1 || ret == -3)
					log_submit(eMessageTypeError, "Couldn't send "
							"firmware(s) to CAN node");

				goto err_delete;
			}
		}

		/* Error condition */
		else {
			log_submit(eMessageTypeError, "Unknown node type (not a "
					"mainboard, slaveboard or microcontroller)");
			ret = -1;
			goto err_delete;
		}

		if (zip_delete_binaries(bindir, &(hnlist->node[i])) == -1) {
			ret = -1;
			goto err;
		}
	}

	goto err;
	/* Reached only if we had error after decompression */
err_delete:
	/*
	 * Useless to check return here, because if this function
	 * is running, that is because we have already an error
	 */
	zip_delete_binaries(bindir, &(hnlist->node[i]));
err:
	return ret;
}

/**
 * @brief Open a file and reads "count" bytes
 * 
 * Read a block of a file (which is opened if currently closed)
 *
 * @param[in,out] wfile struct that contains firmware's path, file
 * descriptor and buffer pointer
 * @param[in] count size of data (in bytes) to read
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_read_file(struct wrapper_file *wfile, int count)
{
	int ret;

	/* If file is not already opened */
	if (wfile->fd == -1) {
		/* Opens file */
		if ((wfile->fd = open(wfile->path, O_RDONLY)) == -1) {
			log_submit_perror(eMessageTypeError, "Cannot open firmware "
					"%s\nopen: ", wfile->path);
			return -1;
		}
		/* Get size of file */
		if ((wfile->size = lseek(wfile->fd, 0, SEEK_END)) == -1) {
			log_submit_perror(eMessageTypeError, "Cannot get size of "
					"firmware %s\nlseek: ", wfile->path);
			return -1;
		}
		/* Go back to the beginning of file */
		if (lseek(wfile->fd, 0, SEEK_SET) == -1) {
			log_submit_perror(eMessageTypeError, "Cannot go back to the "
					"beginning of firmware %s\nlseek: ", wfile->path);
			return -1;
		}
	/* If buffer is already taken (not freed) */
	} else if (wfile->buf != NULL) {
		log_submit(eMessageTypeError, "Buffer for firmware reading "
				"is already taken");
		return -1;
	}

	/* Allocates new buffer */
	wfile->buf = malloc(count+1);
	wfile->buf[count] = '\0';

	/* Read "count" bytes */
	if ((ret = read(wfile->fd, wfile->buf, count)) == -1) {
		log_submit_perror(eMessageTypeError, "Cannot read firmware %s\nread: ",
				wfile->path);
		return -1;
	/*
	 * If read data is less than expected, we
	 * have got end of file. So we close it
	 */
	} else if (ret < count) {
		if (close(wfile->fd) == -1) {
			log_submit_perror(eMessageTypeError, "Cannot close firmware %s\n"
					"close: ", wfile->path);
			return -1;
		}
		wfile->fd = -1;
	}
	return ret;
}

/**
 * @brief Send a mainboard/slaveboard PDO command
 *
 * Called by user to send command return and wait its return
 *
 * @param[in,out] wPDO_ctxt pointer to the PDO state structure
 * @param[in] cmd_word Mainboard/slaveboard PDO command to send
 *
 * @return Return code:
 * *  0: command received
 * * -1: error
 * * -2: cancel wait (typically CTRL + C)
 * * -3: timer expired
 */
int wrapper_send_PDO_command_word(struct wrapper_PDO_context *wPDO_ctxt,
		const unsigned int cmd_word)
{
	wPDO_ctxt->cmd_word = cmd_word;
	wPDO_ctxt->send_cmd_word = true;
	/* Wait code return */
	return wrapper_wait(&condStackOff, &mutex);
}

/**
 * @brief Like wrapper_send_PDO_command_word() but with timeout
 *
 * Called by user to send command return and wait its return,
 * with a expiration time. Used typically to send a first command
 * to a slaveboard/mainboard, because we don't know before if the
 * board support upload protocol (designed for FwUpdater).
 * So, if timer expires, that is because slaveboard/mainboard
 * production firmware doesn't support upload protocol (and doesn't
 * have factory firmware)
 *
 * @param[in,out] wPDO_ctxt pointer to the PDO state structure
 * @param[in] cmd_word Mainboard/slaveboard PDO command to send
 * @param[in] time Timeout (in seconds)
 *
 * @return Return code:
 * *  0: command received
 * * -1: error
 * * -2: cancel wait (typically CTRL + C)
 * * -3: timer expired
 */
int wrapper_send_PDO_command_word_time(struct wrapper_PDO_context *wPDO_ctxt,
		const unsigned int cmd_word, int time)
{
	wPDO_ctxt->cmd_word = cmd_word;
	wPDO_ctxt->send_cmd_word = true;
	/* Wait code return */
	return wrapper_wait_time(&condStackOff, &mutex, time);
}

/**
 * @brief Get the return of a PDO command
 *
 * Called by OPLK/PCIE PDO callback to store command return
 *
 * @param[in,out] wPDO_ctxt pointer to the PDO state structure
 * @param[in] cmd_ret Mainboard/slaveboard PDO command return to store
 */
void wrapper_store_PDO_command_return(struct wrapper_PDO_context *wPDO_ctxt,
		const unsigned int cmd_ret)
{
	wPDO_ctxt->cmd_ret = cmd_ret;
	wPDO_ctxt->get_cmd_ret = false;
	/* End waiting code return */
	pthread_cond_signal(&condStackOff);
}

/**
 * @brief Fake PDO command send, to have SDO download progression return
 *
 * Called by user to check PDO return when SDO upload is in progress. No
 * command word is sent because PDO return is automatically set by FPGA
 * when a SDO upload is in progresss. This function enables PDO listen
 * in order to get SDO return.
 *
 * Especially designed for SDO watching
 *
 * @param[in,out] wPDO_ctxt pointer to the PDO state structure
 * @param[in] cmd_word Mainboard/slaveboard PDO command to "send"
 *
 * @return Return code:
 * *  0: command received
 * * -1: error
 * * -2: cancel wait (typically CTRL + C)
 * * -3: timer expired
 */
int wrapper_check_PDO_upload(struct wrapper_PDO_context *wPDO_ctxt,
		const unsigned int cmd_word)
{
	wPDO_ctxt->get_cmd_ret = true;
	/*
	 * No command word is sent, it is only used for bits
	 * prefix (BSL_CMD_WRITE_CFI or BSL_CMD_WRITE_EPCS)
	 */
	wPDO_ctxt->cmd_word = cmd_word;
	/* Wait code return */
	return wrapper_wait(&condStackOff, &mutex);
}

/**
 * @brief Get string representation of a PDO command
 *
 * Each PDO command owns its string representation in order
 * to print messages. This function provides a string
 * according to the PDO command
 *
 * @param[in] cmd_word Mainboard/slaveboard PDO command
 *
 * @return Pointer to the string representation of a PDO command
 */
const char* wrapper_get_command_word_str(unsigned int cmd_word)
{
	unsigned int i;

	/* For each command word */
	for (i = 0; i < (sizeof(w_cmd_word_debug) / sizeof(*(w_cmd_word_debug))) ; i++) {
		/* Returns debug string of matching command word */
		if (w_cmd_word_debug[i].cmd_bits == cmd_word)
			return w_cmd_word_debug[i].cmd_str;
	}
	return "Invalid command";
}

/**
 * @brief Get string representation of a PDO command return
 *
 * Each PDO command return owns its string representation in order
 * to print messages. This function provides a string according to
 * the PDO command return
 *
 * @param[in] cmd_ret Mainboard/slaveboard PDO command return
 *
 * @return Pointer to the string representation of a PDO command return
 */
const char* wrapper_get_command_return_str(unsigned int cmd_ret)
{
	unsigned int i;

	/* For each command return */
	for (i = 0; i < (sizeof(w_cmd_ret_debug) / sizeof(*(w_cmd_ret_debug))) ; i++) {
		/* Returns debug string of matching command return */
		if (w_cmd_ret_debug[i].cmd_bits == cmd_ret)
			return w_cmd_ret_debug[i].cmd_str;
	}
	return "Invalid return";
}
