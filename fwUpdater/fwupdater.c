#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>
#include <sys/capability.h>
/* For getopt() */
#include <getopt.h>
/* For strlol */
#include <limits.h>
#include <errno.h>
#include <signal.h>

#include "include/fw_context.h"
#include "include/fw_log.h"
#include "include/fw_process.h"
#include "include/fw_wrapper.h"
#include "include/fw_xml.h"
#include "include/fifo.h"
#include "include/input_loop.h"
#include "include/configure_process.h"
#include "include/gui.h"

#define VERSION        1
#define SUBVERSION     1

/* Configuration number to use */
int num_conf = -1;
/* Tell if GUI is enabled or not */
int gui_enabled = 0;
/* Tell if verbose by SSH is enabled or not */
int ssh_enabled = 0;
eMessageType loglevel = eMessageTypeNote;

/**
 * @brief Print the command usage
 *
 * @param[in] name Name of the command
 */
void usage(char *name) {
	printf("Usage: %s [options] -c <[0-9]> xmlfile bindir\n", name);
	printf("  -c  --config\tset used configuration number\n");
	printf("  -h  --help\tprint this usage and exit\n");
	printf("  -g  --gui\tuse graphical interface\n");
	printf("  -s  --ssh\tuse secondary SSH port\n");
}

/**
 * @brief Check the configuration number validity (must be an integer)
 *
 * @param[in] arg String containing the configuration number
 *
 * @return Integer representing the configuration number, -1 else
 */
int arg_check_conf_number(char *arg) {
	char *endptr;
	long val;

	errno = 0;
	val = strtol(arg, &endptr, 10);

	if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN))
			|| (errno != 0 && val == 0)) {
		perror("strtol");
		return -1;
	}

	if (endptr == arg) {
		fprintf(stderr, "No digits were found\n");
		return -1;
	}

	return val;
}

/**
 * @brief Parse all arguments
 *
 * @param[in] argc Argument counter
 * @param[in] argv Argument vector
 *
 * @return 0 if succeeded, -1 else
 */
int arg_check(int argc, char **argv) {
	int opt;
	static struct option longopts[] = {
		{"conf", required_argument, 0, 'c'},
		{"gui", no_argument, 0, 'g'},
		{"help", no_argument, 0, 'h'},
		{"ssh", no_argument, 0, 's'},
		{0, 0, 0, 0}};

	while ((opt = getopt_long(argc, argv, "c:ghs", longopts, NULL)) != -1) {
		switch (opt) {
			case 'c':
				num_conf = arg_check_conf_number(optarg);
				if (num_conf == -1)
					return -1;

				break;
			case 'g':
				gui_enabled = 1;
				break;
			case 'h':
				usage(argv[0]);
				break;
			case 's':
				ssh_enabled = 1;
				break;
			default:
				return -1;
		}
	}
	if (gui_enabled && ssh_enabled) {
		fprintf(stderr, "Missing configuration number for firmwares\n");
		return -1;
	}

	if (num_conf == -1 && !gui_enabled) {
		fprintf(stderr, "GUI and secondary SSH port are mutually exclusive\n");
		return -1;
	}

	if (gui_enabled)
		log_submit(eMessageTypeNote, "GUI enabled");
	else if (ssh_enabled)
		log_submit(eMessageTypeNote, "Secondary SSH port enabled");

	return 0;
}

/**
 * @brief Main function
 *
 * @param[in] argc Argument counter
 * @param[in] argv Argument vector
 *
 * @return 0 if succeeded, 1 else
 */
int main(int argc, char **argv) {
	char    *bindir = NULL, *xmlfile = NULL;
	int     ret = EXIT_FAILURE;
	struct hardware_node_list   *nodes = NULL;
	struct wrapper_context_list *wrapper_contexts = NULL;
    int     input,  output;
    struct sigaction    act;

	if (log_open("fwupdater.log") == -1) {
		fprintf(stderr, "%s: cannot open log file\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	log_submit(eMessageTypeNote, "FwUpdater version %lu.%lu",
			VERSION, SUBVERSION);

	process_enable_coredump();

	/* Check options */
	if (arg_check(argc, argv) == -1) {
		usage(argv[0]);
		ret = EXIT_SUCCESS;
		goto exit_log;
	}

	/*
	 * We don't designate XML configuration file and
	 * binaries' directory in terminal when we use
	 * graphical interface
	 */
	if (gui_enabled) {
        memset(&act, 0, sizeof(act));
        act.sa_handler = SIG_IGN;
        sigaction(SIGPIPE, &act, NULL);
        if (!init_fifos() || !gui_launch() || !open_fifos(&input, &output)) {
            ret = EXIT_FAILURE;
            goto exit_log;
        }
        configure_process(output);
        input_loop(input, output);
        close_fifos(input, output);
        ret = EXIT_SUCCESS;
        goto exit_log;
	} else {
		/* Check mandatory arguments' presence */
		if ((argc - optind) < 2) {
			log_submit(eMessageTypeCritical, "Missing argument(s)");
			usage(argv[0]);
			goto exit_log;
		}

		xmlfile = argv[optind];
		bindir = argv[optind + 1];

		/* Check existence and regularity of XML file */
		if ((process_file_is_regular(xmlfile)) == -1) {
			log_submit_perror(eMessageTypeError, "XML file %s unusable",
					xmlfile);
			goto exit_log;
		}

		/* Check existence of binaries' directoy */
		if ((process_file_is_directory(bindir)) == -1) {
			log_submit_perror(eMessageTypeError, "Directory %s unusable",
					xmlfile);
			usage(argv[0]);
			goto exit_log;
		}

		/*
		 * Get structures to store hardwares nodes
		 * informations and wrappers's contexts
		 */
		if ((nodes = context_initialize_hardware()) == NULL ||
				(wrapper_contexts = context_initialize_wrapper()) == NULL)
			goto exit_log;

		/* Store in hardware context the number configuration */
		nodes->number_conf = num_conf;

		/*
		 * Parse XML file to have a structural memory
		 * representation of hardware and wrapper context
		 */
		if (xml_get_context(xmlfile, nodes, wrapper_contexts, true) == -1)
			goto exit_free;

		/* Find archive of each hardware node */
		if (context_get_archive_names(nodes, bindir, true) == -1)
			goto exit_free;
	}

	/* Check 'oplki210mn'/'mhp' kernel modules */
	if (process_check_kernel_modules_use(wrapper_contexts) == -1) {
		log_submit(eMessageTypeCritical, "Error in kernel modules "
				"dependencies");
		goto exit_free;
	}
	/* Check if process can modify schedule settings */
	if (process_check_flag(CAP_SYS_NICE) == -1) {
		log_submit(eMessageTypeCritical, "CAP_SYS_NICE capability "
				"unavailable");
		goto exit_free;
	}
	/* Modify schedule settings and signals */
	if (process_sched_init() == -1) {
		log_submit(eMessageTypeCritical, "Unable to modify schedule "
				"settings or signals");
		goto exit_free;
	}

	/* Print all informations on current context, if we are in debug mode */
	if (loglevel == eMessageTypeDebug)
		context_print_global_context(nodes, wrapper_contexts);

	if (wrapper_initialize(nodes, wrapper_contexts, bindir, NULL) != 0) {
		log_submit(eMessageTypeCritical, "Wrappers initialization aborted");
		goto exit_free;
	}

	if (wrapper_send_firmwares(nodes, wrapper_contexts, bindir, NULL) != 0)
		goto exit_free;

	log_submit(eMessageTypeNote, "Verifying all nodes have been updated");
	if (wrapper_verify(nodes, wrapper_contexts, bindir) != 0) {
		log_submit(eMessageTypeCritical, "Wrappers verification aborted");
		goto exit_free;
	}

	log_submit(eMessageTypeNote, "Success!");
	ret = EXIT_SUCCESS;

exit_free:
	/* Shutdown all wrappers */
	wrapper_free(wrapper_contexts);
	/* Erase all memory structures of context */
	context_free(nodes, wrapper_contexts);
exit_log:
	/* Close log */
	log_close();

	exit(ret);
}
