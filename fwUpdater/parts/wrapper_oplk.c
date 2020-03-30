#include "include/fw_wrapper_oplk.h"

/*
 * * * * * * * *
 * OPLK Wrapper
 * * * * * * * *
 */

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

/**
 * @brief Reset OPLK context, used when GUI is enabled
 *
 * Reinitialize OPLK context, allowing multiple consecutive updates with GUI
 */
void wrapper_oplk_reset_context(void)
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
 * @brief OPLK event callback
 * 
 * Callback registered in the OPLK wrapper. This function will be called when
 * interesting events happens in the OPLK wrapper. It must respect this
 * prototype to be a valid callback
 *
 * @param[in] evt the event which appended
 * @param[in] data the event data associated. Mostly NULL, check
 * eventCallback.h for details
 */
void wrapper_oplk_events_callback(const oplkEventType evt, const void *data)
{
	/* Used to periodically print SDO uploading */
	static int c = 0;
	//printf("Got an oplk event: '%s'\n", debugstr_getOplkWrapperEventStr(evt));

	if (evt == OPLK_WRAPPER_SDO_TRANSFER_OK ||
			evt == OPLK_WRAPPER_SDO_TRANSFER_FAILED) {
		const tSdoComFinished *sdoComFinished = (const tSdoComFinished *)data;
		if (evt == OPLK_WRAPPER_SDO_TRANSFER_OK) {
			if (c == 100) {
				c = 0;
				fprintf(stderr, "SDO transfer finished : nodeId:%u (0x%X/%u) "
						"abordCode:'%s' transferredBytes:%u\n",
						sdoComFinished->nodeId,
						sdoComFinished->targetIndex,
						sdoComFinished->targetSubIndex,
						debugstr_getAbortCodeStr(sdoComFinished->abortCode),
						sdoComFinished->transferredBytes);
			} else {
				c++;
			}
		} else {
			fprintf(stderr, "SDO transfer failed : nodeId:%u (0x%X/%u) "
					"abordCode:'%s' transferredBytes:%u\n",
					sdoComFinished->nodeId,
					sdoComFinished->targetIndex,
					sdoComFinished->targetSubIndex,
					debugstr_getAbortCodeStr(sdoComFinished->abortCode),
					sdoComFinished->transferredBytes);
		}

		wrapper_SDO_ctxt.sent = sdoComFinished->transferredBytes;
		if (evt == OPLK_WRAPPER_SDO_TRANSFER_FAILED)
			wrapper_SDO_ctxt.error = sdoComFinished->abortCode;

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
 * @brief Callback function processing openPowerlink PDOs
 *
 * Callback registered in the OPLK wrapper to be notified when we can do PDO
 * related things
 *
 * Called by OpenPowerlink stack
 *
 * @param[in] piOut Area memory to read (slaveboard -> FwUpdater)
 * @param[in] piIn Area memory to write (FwUpdater -> slaveboard)
 * @param[in] sizeIn Sizeof of input area memory
 * @param[in] sizeOut Sizeof of output area memory
 */
void wrapper_oplk_ProcessPdoSyncCb(void* piIn, const void* piOut,
			uint32_t sizeIn,
			uint32_t sizeOut,
			struct timeval* const currentCycleTv,
			struct timeval* const nextCycleTv)
{
	if (wrapper_PDO_ctxt.send_cmd_word == true) {
		/* No new command word to send for now */
		wrapper_PDO_ctxt.send_cmd_word = false;
		/* Write command word in PDO */
		((PI_IN*)piIn)->s_TPDO_generalsystem[0] = wrapper_PDO_ctxt.cmd_word;
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
		if ((((PI_OUT*)piOut)->s_RPDO_generalsystem[0] & BSL_CMD_FIELD_MSK) == wrapper_PDO_ctxt.cmd_word) {
			/* If there is error (BSL_STAT_ERR_CODE_MSK bits different from 0) */
			if ((((PI_OUT*)piOut)->s_RPDO_generalsystem[0] & BSL_STAT_ERR_CODE_MSK) != 0) {
				log_submit(eMessageTypeNote, "PDO return : \"%s\" (0x%X)",
						wrapper_get_command_return_str(((PI_OUT*)piOut)->s_RPDO_generalsystem[0] & BSL_STAT_ERR_CODE_MSK),
						((PI_OUT*)piOut)->s_RPDO_generalsystem[0] & BSL_STAT_ERR_CODE_MSK);
				wrapper_PDO_ctxt.error = true;
			}
			wrapper_store_PDO_command_return(&wrapper_PDO_ctxt,
					((PI_OUT*)piOut)->s_RPDO_generalsystem[0]);
		/*
		 * High 16 bits of return command doesn't match,
		 * probably because command is refused. Checks
		 * in corresponding bits if it is the case
		 */
		} else if ((((PI_OUT*)piOut)->s_RPDO_generalsystem[0] & BSL_STAT_CMD_REFUSED) == BSL_STAT_CMD_REFUSED) {
			log_submit(eMessageTypeError, "PDO return : \"Command refused\" (0x%X)", ((PI_OUT*)piOut)->s_RPDO_generalsystem[0]);
			wrapper_PDO_ctxt.error = true;
			wrapper_store_PDO_command_return(&wrapper_PDO_ctxt,
					((PI_OUT*)piOut)->s_RPDO_generalsystem[0]);
		}
	}
	UNUSED(piIn);
	UNUSED(piOut);
	UNUSED(sizeIn);
	UNUSED(sizeOut);
	UNUSED(currentCycleTv);
	UNUSED(nextCycleTv);
}

/** 
 * @brief Initialize the Openpowerlink stack
 *
 * Initialize the Openpowerlink stack with some default values, the defined
 * 'mnobd.cdc' file and a cycle. If the stack fails to start, the function
 *  will try again (a shutdown is done for cases when the stack is not
 *  previously correctly halted).
 *
 * @param[in] mnobd Path to the 'mnobd.cdc' file to use
 * @param[in] cycle Cycle (in milliseconds) to apply to the stack
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_oplk_initialize(char *mnobd, int cycle)
{
	wOplkInitParam oplkParams;
	tOplkError oplkRet;

	memset(&oplkParams, 0, sizeof(wOplkInitParam));
	oplkParams.cycleLenUs              = cycle; // 0 = use the one from the cdc file
	oplkParams.processImageInSize      = sizeof(PI_IN);
	oplkParams.processImageOutSize     = sizeof(PI_OUT);
	oplkParams.cdcFilename             = mnobd;
	oplkParams.callbackEvent           = wrapper_oplk_events_callback;
	oplkParams.callbackProcessSync     = wrapper_oplk_ProcessPdoSyncCb;
	oplkParams.multiplCylceCnt         = 0;
	//oplkParams.pcapDevName             = "veth0"; // hardcoded interface name for now

	log_submit(eMessageTypeNote, "Start OpenPowerlink wrapper "
			"with .cdc file %s", mnobd);
	/*
	 * OPLK wrapper starting has several tries. This manages the case where
	 * previous OPLK execution were not correctly halted
	 */
	for (int try = 0 ; try < 2 ; try++) {
		/* Init the OpenpowerLink stack with the information provided */
		if ((oplkRet = wOplkInit(&oplkParams)) == kErrorOk) {
			/*
			 * Wait some time to let the stack
			 * find & sync controlled nodes
			 */
			log_submit(eMessageTypeNote, "Wait OpenPowerlink nodes finding "
					"and synchronisation");
			if (wrapper_wait_time(&condStackOff, &mutex, 15) == -2) {
				return -2;
			}
			return 0;
		}

		/*
		 * If this code is executed, that means that OpenpowerLink
		 * stack initialization has failed
		 */

		if (oplkRet == kErrorNoResource)
			log_submit(eMessageTypeError, "An error happend while starting "
					"wrapper oplk : \"%s\" (0x%04x)\nMaybe OpenPowerlink nodes "
					"are not wired, or another process currently use network",
					debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
		else
			log_submit(eMessageTypeError, "An error happend while starting "
					"wrapper oplk : \"%s\" (0x%04x)",
					debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
		/*
		 * An error has occured, log it and halt
		 * previous OpenPowerlink execution
		 */
		if (try == 0) {
			log_submit(eMessageTypeError, "Shutdown previous OpenPowerlink "
					"execution, retry launching");
			if ((oplkRet = shutdownPowerlink()) != kErrorOk) {
				log_submit(eMessageTypeError, "Error in shutdown of "
						"OpenPowerLink wrapper : \"%s\" (0x%04x)",
						debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
				return -1;
			}
		}
	}
	return -1;
}

/**
 * @brief Scan the openPowerlink network for CN and get their version
 *
 * Wait for full detection of OpenPowerlink nodes, then check for each node
 * its hardware/software version and configuration with defined nodes in
 * XML file. Update hardware nodes list to add in OpenPowerlink user
 * nodes their network ID
 *
 * @param[in] hnlist hardware nodes list
 * @param[in] verify If 'true', raises an error if any node is not up to date
 * @param[in] prog_trk (GUI) structure used to follow a firmware update
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_oplk_scan_network(struct hardware_node_list *hnlist, int verify,
		progress_tracker_t *prog_trk)
{
	int i, j, oplk_count = 0, synced_oplk_count = 0;
	int comparative_version;
	char *conf_str;
	/* Details about nodes found */
	const NodeInfoArray* found_nodes = NULL;
	tOplkError oplkRet;

	/*
	 * This function will return kErrorOk only if all nodes in the cdc where
	 * found and synced. If one node was synced but lead to an error, its
	 * state will be set accordingly
	 */
	oplkRet = wOplkIdentifyNodes(&found_nodes);
	/*
	 * We check if 'oplkRet == kErrorSdoSeqInvalidEvent' because this allows
	 * to continue still there are missing nodes (may be wanted : we have a
	 * full nodes .cdc file, but we can have only a subpart of these nodes)
	 */
	if (((oplkRet == kErrorOk) || (oplkRet == kErrorSdoSeqInvalidEvent))
			&& found_nodes != NULL) {
		if (oplkRet == kErrorSdoSeqInvalidEvent)
			log_submit(eMessageTypeWarning, "Scanning OpenPowerLink network "
					"with some nodes not synchronized");

		/* Count user defined nodes using OpenPowerlink */
		for (j = 0; j < hnlist->count ; j++) {
			/* Node has to be flashed and is an OpenPowerlink one */
			if (hnlist->node[j].to_update &&
					(hnlist->node[j].bustype == eBusTypeEPL))
				oplk_count++;
		}
		/*
		 * If there are not enough detected OpenPowerlink nodes
		 * (according to number of GUI/XML defined nodes)
		 */
		if ((int)found_nodes->size < oplk_count) {
			log_submit(eMessageTypeError, "There are more defined nodes in "
					"XML file or GUI interface (%d nodes) than detected "
					"nodes in OpenPowerlink CDC file (%d nodes)",
					(int)found_nodes->size, oplk_count);
			return -1;
		}

		/* Count synced found nodes */
		for (i = 0; i < (int)found_nodes->size ; ++i) {
			if (found_nodes->nodes[i].state == SYNCED)
				synced_oplk_count++;

		}
		log_submit(eMessageTypeNote, "Defined OpenPowerLink nodes (in XML):%d "
				"(in CDC):%d (detected):%d", oplk_count, (int)found_nodes->size,
				synced_oplk_count);

		/* For each found node */
		for (i = 0; i < (int)found_nodes->size ; ++i) {
			/*
			 * Only OpenPowerLink nodes at state "SYNCED"
			 * or "FOUND_CDC" are expected
			 * Important:
			 * - SYNCED: existing CN, ready to work
			 * - FOUND_CDC:
			 *   - non-existing CN, only defined in .cdc file
			 *   - failed existing CN
			 */
			if ((found_nodes->nodes[i].state != SYNCED) &&
					(found_nodes->nodes[i].state != FOUND_CDC)) {
				log_submit(eMessageTypeError, "OpenPowerlink node %d is not "
						"in synced state (state:%d)",
						found_nodes->nodes[i].nodeID,
						found_nodes->nodes[i].state);
				return -1;
			}

			/* For each user defined node */
			for (j = 0; j < hnlist->count ; j++) {
				/*
				 * TODO: check "device_name", that may defined to "Stago
				 * SlaveBoard". So, check that string and verify if
				 * matching node is a Slaveboard. If not a
				 * slaveboard, continue loop
				 */

				/*
				 * Only defined nodes using OpenPowerLink
				 * bus are compared
				 */
				if (hnlist->node[j].bustype != eBusTypeEPL)
					continue;

				/*
				 * If foreach-ed defined node doesn't have
				 * same OpenpowerLink id than found node
				 */
				if (hnlist->node[j].oplk_idnode !=
						found_nodes->nodes[i].nodeID) {
					continue;
				}

				/* If node shouln't to be flashed */
				if (!hnlist->node[j].to_update)
					break;

				if (found_nodes->nodes[i].state == FOUND_CDC) {
					log_submit(eMessageTypeError, "OpenPowerlink node %d "
							"is not in synced state (stayed in FOUND_CDC "
							"state)", found_nodes->nodes[i].nodeID);
					return -1;
				}
				/*
				 * Hardware version has to be of the form "A01 Conf3". First
				 * search in user defined nodes the one that has by example
				 * the pattern "A01"
				 */
				if (strncmp(found_nodes->nodes[i].hwVersion, hnlist->node[j].name, 3) == 0) {
					/*
					 * Get configuration number in search pattern "Conf" (that
					 * is followed by a digit)
					 */
					if ((conf_str = strstr(found_nodes->nodes[i].hwVersion, "Conf")) == NULL) {
						log_submit(eMessageTypeError, "Bad hardware name "
								"format (%s) for node %.3s",
								found_nodes->nodes[i].hwVersion,
								found_nodes->nodes[i].hwVersion);
						return -1;
					}
				} else {
					/*
					 * Pattern doesn't match, maybe because device is in
					 * factory mode. So tests typical factory pattern
					 * Note: '-1' is to remove implicit '\0' testing (because
					 * sizeof() counts size of string + the '\0')
					 */
					if (strncmp(found_nodes->nodes[i].hwVersion, "FACT", sizeof("FACT")-1) == 0) {
						/*
						 * Device is already in factory mode. So we won't
						 * send command word to reset device in factory mode
						 * and won't restart OpenPowerlink wrapper to detect
						 * changing device mode
						 */
						hnlist->node[j].is_factory_mode = true;
						log_submit(eMessageTypeNote, "Found node: [id:%d "
								"factory_version:%s] -> is already in factory "
								"mode, upgrade to [conf:%d version:%s]",
								found_nodes->nodes[i].nodeID,
								found_nodes->nodes[i].swVersion,
								hnlist->node[j].conf,
								hnlist->node[j].version);

						/*
						 * If this function is used to verify devices mode, so
						 * the nominal case is that no device is supposed to
						 * be in factory mode
						 */
						if (verify) {
							log_submit(eMessageTypeError, "Node is not "
									"supposed to be in factory mode");
							return -1;
						} else {
							/*
							 * Device was already in factory mode
							 * when FwUpdater started
							 */
							hnlist->node[j].was_factory_mode = true;
						}
						/*
						 * Because software version returned by CN node in
						 * factory node is the factory version, doesn't
						 * compare versions
						 */
						break;
					} else {
						log_submit(eMessageTypeError, "Bad hardware name "
								"format (\"%s\") for node %s [oplkid:%d]."
								" Expected: \"%s ConfX\" or string "
								"starting with \"FACT\"",
								found_nodes->nodes[i].hwVersion,
								hnlist->node[j].name,
								hnlist->node[j].oplk_idnode,
								hnlist->node[j].name);
						return -1;
					}
				}
				/*
				 * Compare software version between XML version
				 * and got version (in OpenPowerlink node)
				 */
				comparative_version = context_compare_version(
						hnlist->node[j].version,
						found_nodes->nodes[i].swVersion);
				/*
				 * If configuration number is different but XML and got
				 * versions are same, so we have to flash the device
				 *
				 * Note: conf_str[4] is to point after "Conf"
				 */
				if ((atoi(&(conf_str[4])) != hnlist->node[j].conf)
						&& (comparative_version == 0))
					comparative_version = 3;

				switch(comparative_version) {
					/*
					 * If XML defined version of node is equals to
					 * got version, useless to upgrade firmware
					 */
					case 0:
						/* Disable firmware node updating */
						hnlist->node[j].to_update = false;

						log_submit(eMessageTypeNote, "Found node: [name:%.3s "
								"id:%d conf:%d version:%s] -> no need to flash"
								" (same version)",
								found_nodes->nodes[i].hwVersion,
								found_nodes->nodes[i].nodeID,
								atoi(&(conf_str[4])),
								found_nodes->nodes[i].swVersion);
                        if (prog_trk != NULL) {
                            if (!prog_trk->ignore_trans_sb ||
                                hnlist->mc_oplk_node != &hnlist->node[j]) {
                                prog_trk->package = hnlist->node[j].archive;
                                notify_no_need(prog_trk);
                            }
                        }
						break;
					case 1:
						log_submit(eMessageTypeNote, "Found node: [name:%.3s "
								"id:%d conf:%d version:%s] -> upgrade to "
								"[conf:%d version:%s]",
								found_nodes->nodes[i].hwVersion,
								found_nodes->nodes[i].nodeID,
								atoi(&(conf_str[4])),
								found_nodes->nodes[i].swVersion,
								hnlist->node[j].conf,
								hnlist->node[j].version);
						break;
					case 2:
						log_submit(eMessageTypeNote, "Found node: [name:%.3s "
								"id:%d conf:%d version:%s] -> downgrade to "
								"[conf:%d version:%s]",
								found_nodes->nodes[i].hwVersion,
								found_nodes->nodes[i].nodeID,
								atoi(&(conf_str[4])),
								found_nodes->nodes[i].swVersion,
								hnlist->node[j].conf,
								hnlist->node[j].version);
						break;
					case 3:
						log_submit(eMessageTypeNote, "Found node: [name:%.3s "
								"id:%d conf:%d version:%s] -> change "
								"configuration to [conf:%d version:%s]",
								found_nodes->nodes[i].hwVersion,
								found_nodes->nodes[i].nodeID,
								atoi(&(conf_str[4])),
								found_nodes->nodes[i].swVersion,
								hnlist->node[j].conf,
								hnlist->node[j].version);
						break;
					case -1:
						log_submit(eMessageTypeError, "Bad software version "
								"format (%s) for node %.3s",
								found_nodes->nodes[i].swVersion,
								found_nodes->nodes[i].hwVersion);
						return -1;
						break;
				}
				/* Nodes are supposed to be up to date */
				if (verify && (comparative_version != 0)) {
					log_submit(eMessageTypeError, "While verifying that all "
							"nodes were updated: at least one node is not "
							"up to date (see node description above) - "
							"code %d", comparative_version);
					return -1;
				}
				break;
			}
			/*
			 * It exist a ready-to-work OpenPowerlink node
			 * not defined in XML file
			 */
			if (j == hnlist->count && found_nodes->nodes[i].state == SYNCED)
				log_submit(eMessageTypeNote, "Ignored node %d (doesn't exist "
						"in XML file). State:%d", found_nodes->nodes[i].nodeID,
						found_nodes->nodes[i].state);
		}
	}

	return 0;
}

/**
 * @brief Switch (toggle) the firmware mode of a slaveboard
 *
 * Switch a OpenPowerLink node to its alternate firmware mode. Manage the
 * restart OpenPowerLink wrapper. That last one has to be beforehand
 * initialized
 *
 * @param[in] hnnode hardware node structure
 * @param[in] woplk_ctxt OpenPowerLink wrapper context (cycle,
 * time sleeping, state)
 * @param[in] mnobd_path path of the 'mnobd.cdc' file to use
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_oplk_switch_factory(struct hardware_node *hnnode,
		struct wrapper_context *woplk_ctxt, char *mnobd_path)
{
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
			log_submit(eMessageTypeError, "No response in firmware switch mode "
					"command to node %d (%s). Maybe it doesn't support upload "
					"protocol and doesn't have factory firmware",
					hnnode->oplk_idnode, hnnode->name);
		return ret;
	}
	if (wrapper_PDO_ctxt.error) {
		log_submit(eMessageTypeError, "Error in PDO operations");
		return -1;
	}

	/* Switch to factory command word */
	if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_SWITCH_TO_FACTORY) == -2) {
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
	if (wrapper_oplk_initialize(mnobd_path, woplk_ctxt->cycle) == -1) {
		log_submit(eMessageTypeError, "OPLK wrapper cannot be loaded");
		return -1;
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


#define SIZE_OF_FIRMWARE_SDO 1130
/**
 * @brief Send to a OpenPowerLink node all firmwares associated to it
 *
 * Manages :
 * * PDO commands to OpenPowerLink node (user/factory mode switching,
 *   imminent file sending notification, etc...)
 * * File reading and sending
 * * Timers
 *
 * @param[in] hnnode hardware node structure
 * @param[in] woplk_ctxt OpenPowerLink wrapper context (cycle,
 * time sleeping, state)
 * @param[in] bindir directory where to find firmware binaries
 * @param[in] prog_trk (GUI) structure used to follow a firmware update
 *
 * @return 0 if succeeded, -1 else
 */
int wrapper_oplk_send_firmwares(struct hardware_node *hnnode,
		struct wrapper_context *woplk_ctxt, char *bindir,
		progress_tracker_t *prog_trk)
{
	int                     	b, ret;
	char                    	fw_path[PATH_MAX];
	char mnobd_path[PATH_MAX];
	/*
	 * Used to periodically check SDO uploading
	 * state by watch PDO return code
	 */
	int c;
	/*
	 * In XML file, internal ZIP archive paths may have
	 * a '/'. But only name interests us. That variable
	 * contains a pointer in filename
	 */
	tOplkError                  oplkRet;
	const NodeInfoArray*		found_nodes = NULL;
	struct wrapper_file			wfile;
	int                     	read_bytes = 0, total_read_bytes = 0;
	struct hardware_node_file	*hnfile;
	/* To time each firmware uploading */
	struct timeval tv_begin, tv_end;
	char *str_time;
    /* For progression print */
    struct upload_percent percent;
    /* To get binary size */
    struct stat st;

	/* Loading of slaveboard .cdc */
	/* Concatenate firmware name with binaries' directory */
	if (snprintf(mnobd_path, PATH_MAX, "%s/%s", bindir, hnnode->sb_binary) >= PATH_MAX) {
		log_submit(eMessageTypeError, "Path too long");
		return -1;
	}

	/* Check existence and regularity of .cdc file */
	if ((process_file_is_regular(mnobd_path)) == -1) {
		log_submit_perror(eMessageTypeError, ".cdc file %s unusable",
				mnobd_path);
		return -1;
	}

	/* Initialize OpenPowerLink wrapper */
	if (wrapper_oplk_initialize(mnobd_path, woplk_ctxt->cycle) == -1) {
		log_submit(eMessageTypeError, "OPLK wrapper cannot be loaded");
		return -1;
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

	if (prog_trk != NULL)
		prog_trk->iteration_count = hnnode->files_count;

	/* For each node's firmware to send */
	for (b = 0; b < hnnode->files_count ; b++) {
		hnfile = &hnnode->files[b];

		/* Concatenate firmware name with binaries directory */
		if (snprintf(fw_path, PATH_MAX, "%s/%s", bindir, hnfile->base_name) >= PATH_MAX) {
			log_submit(eMessageTypeError, "Path too long");
			return -1;
		}

		if (hnfile->flash_type == eFlashTypeEPCS)
			log_submit(eMessageTypeNote, "Send file %s to node %d (%s) in EPCS "
					"memory", fw_path, hnnode->oplk_idnode, hnnode->name);
		else
			log_submit(eMessageTypeNote, "Send file %s to node %d (%s) in CFI "
					"memory", fw_path, hnnode->oplk_idnode, hnnode->name);

		/* If node is not in factory mode */
		if (!hnnode->is_factory_mode) {
			log_submit(eMessageTypeNote, "Switch node %d (%s) to factory mode",
					hnnode->oplk_idnode, hnnode->name);
			ret = wrapper_oplk_switch_factory(hnnode, woplk_ctxt, mnobd_path);
			if (ret)
				return ret;
		}

		/* Warn of an upcoming flash command word */
		if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_PREPARE_TO_BSL) == -2)
			return -2;
		if (wrapper_PDO_ctxt.error) {
			log_submit(eMessageTypeError, "Error in PDO operations");
			return -1;
		}
		if (hnfile->flash_type == eFlashTypeEPCS) {
			/* Send firmware to EPCS memory command word */
			if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_WRITE_EPCS) == -2)
				return -2;
		} else {
			/* Send firmware to CFI memory command word */
			if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_WRITE_CFI) == -2)
				return -2;
		}
		if (wrapper_PDO_ctxt.error) {
			log_submit(eMessageTypeError, "Error in PDO operations");
			return -1;
		}

		memset(&wfile, 0, sizeof(wfile));
		c = 0;
		read_bytes = SIZE_OF_FIRMWARE_SDO;
		total_read_bytes = 0;
		wfile.fd = -1;
		wfile.path = fw_path;
		wfile.buf = NULL;

        /* Get the stat struct of the binary (to later get its size) */
        if (stat(fw_path, &st) == -1) {
            log_submit_perror(eMessageTypeError, "Couln't get the size "
					"of binary\nstat: ");
            return -1;
        }

        /* Initialize a percentage structure */
        log_init_percent_struct(&percent, st.st_size, SIZE_OF_FIRMWARE_SDO);

		/* Start timing */
		if (gettimeofday(&tv_begin, NULL) == -1) {
			log_submit(eMessageTypeError, "Unable to get time\ngettimeofday: ");
			return -1;
		}

		while (read_bytes == SIZE_OF_FIRMWARE_SDO) {
			if ((read_bytes = wrapper_read_file(&wfile, SIZE_OF_FIRMWARE_SDO)) == -1) {
				return -1;
				/*
				 * If size file is a multiple of SIZE_OF_FIRMWARE_SDO, the last
				 * sending will send 0 bytes. So, if size to send is 0, skip
				 */
			} else if (read_bytes == 0) {
				free(wfile.buf);
				break;
			}
			total_read_bytes += read_bytes;

            percent.counter++;

			if ((oplkRet = wOplkWriteSdo(hnnode->oplk_idnode, 0x5F50, 1,
							wfile.buf, read_bytes, &oplkRet)) != kErrorApiTaskDeferred) {
				log_submit(eMessageTypeError, "[%d/%d] Error in uploading SDO "
						"at 0x%X/0x%X of OpenPowerLink node \"%s\" : \"%s\" "
						"(0x%04x)", total_read_bytes, wfile.size, 0x5F50,
						0x1, hnnode->name,
						debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
				return -1;
			}

			/* Wait end of sending */
			if (wrapper_wait(&condStackOff, &mutex) == -2)
				return -2;

			if (wrapper_SDO_ctxt.error != 0 || wrapper_SDO_ctxt.sent == 0) {
				log_submit(eMessageTypeError, "[%d/%d] Error in uploading SDO "
						"at 0x%X/0x%X of OpenPowerLink node \"%s\" : abortCode:"
						"0x%04x sentBytes:%d", total_read_bytes, wfile.size,
						0x5F50, 0x1, hnnode->name, wrapper_SDO_ctxt.error,
						wrapper_SDO_ctxt.sent);
				free(wfile.buf);
				wfile.buf = NULL;
				return -1;
			}

			/*
			 * Every two SDO, a check is done in PDO return to
			 * verify there is no error. We base in the fact when one
			 * error is returned by FPGA, it will be always present
			 * (even if next SDO are successful). So it is not disturbing
			 * to check only one time on two
			 */
			if (c == 2) {
				c = 0;
				if (hnfile->flash_type == eFlashTypeEPCS) {
					/* Check upload state to EPCS memory */
					if (wrapper_check_PDO_upload(&wrapper_PDO_ctxt, BSL_CMD_WRITE_EPCS) == -2)
						return -2;
				} else {
					/* Check upload state to CFI memory */
					if (wrapper_check_PDO_upload(&wrapper_PDO_ctxt, BSL_CMD_WRITE_CFI) == -2)
						return -2;
				}
				if (wrapper_PDO_ctxt.error) {
					log_submit(eMessageTypeError, "Error in PDO operations");
					return -1;
				}
			} else {
				c++;
			}

			free(wfile.buf);
			wfile.buf = NULL;

			/* If we have sent enough blocks to print a message */
			if (percent.counter == percent.threshold) {
				/* Reinitialize counter */
				percent.counter = 0;
				percent.cur_percent += percent.slice_percent;

				/*
				 * Do not print when we are at 100%, because the mathematical
				 * formula is not very precise (there are still blocks to
				 * be sent)
				 */
				if (percent.cur_percent < 100) {
					fprintf(stderr, "[%.2d%%] Uploaded part to node "
							"slaveboard %s\n", percent.cur_percent,
							hnnode->name);
					if (prog_trk != NULL)
						notify_progress(prog_trk, percent.cur_percent);
				}
			}
		}

		/* Stop timing */
		if (gettimeofday(&tv_end, NULL) == -1) {
			log_submit(eMessageTypeError, "Unable to get time\ngettimeofday: ");
			return -1;
		}
		if ((str_time = process_time_tostr(tv_begin, tv_end)) == NULL) {
			log_submit(eMessageTypeError, "Unable to get elapsed time "
					"to upload file %s", fw_path);
			return -1;
		}

		log_submit(eMessageTypeNote, "[100%%] Uploading file %s done. "
				"Elapsed time: %s", fw_path, str_time);
		free(str_time);

		if (prog_trk != NULL)
			prog_trk->iteration_index += 1;
	}
	/* Warn of a new command word */
	if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_PREPARE_TO_BSL) == -2)
		return -2;

	if (wrapper_PDO_ctxt.error) {
		log_submit(eMessageTypeError, "Error in PDO operations");
		return -1;
	}
	/* Validate firmware sent command word */
	if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_VALIDATE_BSL) == -2)
		return -2;

	if (wrapper_PDO_ctxt.error) {
		log_submit(eMessageTypeError, "Error in PDO operations");
		return -1;
	}
	/* Warn of a new command word */
	if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_PREPARE_TO_BSL) == -2)
		return -2;

	if (wrapper_PDO_ctxt.error) {
		log_submit(eMessageTypeError, "Error in PDO operations");
		return -1;
	}
	/* Come back to user mode command word */
	if (wrapper_send_PDO_command_word(&wrapper_PDO_ctxt, BSL_CMD_SWITCH_TO_USER) == -2)
		return -2;

	if (wrapper_PDO_ctxt.error) {
		log_submit(eMessageTypeError, "Error in PDO operations");
		return -1;
	}

	log_submit(eMessageTypeNote, "All files sent to node %d (%s)",
			hnnode->oplk_idnode, hnnode->name);

    /* For future updates */
    hnnode->is_factory_mode = false;

    /* Halt OpenPowerLink bus */
	if ((oplkRet = shutdownPowerlink()) != kErrorOk) {
		log_submit(eMessageTypeError, "Error in shutdown of OpenPowerLink "
				"wrapper : \"%s\" (0x%04x)",
				debugstr_getOplkWrapperRetValStr(oplkRet), oplkRet);
		return -1;
	}
	/* The wrapper is now in state "off" */
	woplk_ctxt->is_on = false;

	return 0;
}
