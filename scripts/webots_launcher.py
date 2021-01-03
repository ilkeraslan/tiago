#!/usr/bin/env python

"""Start Webots."""

import optparse
import os
import sys
import subprocess

optParser = optparse.OptionParser()
optParser.add_option("--world", dest="world", default="", help="Path to the world to load.")
optParser.add_option("--mode", dest="mode", default="realtime", help="Startup mode.")
optParser.add_option("--no-gui", dest="noGui", default="false", help="Start Webots with minimal GUI.")
options, args = optParser.parse_args()

if 'WEBOTS_HOME' not in os.environ:
    sys.exit('WEBOTS_HOME environment variable not defined.')
command = [os.path.join(os.environ['WEBOTS_HOME'], 'webots'), '--mode=' + options.mode, options.world]
if options.noGui.lower() == 'true':
    command.append('--stdout')
    command.append('--stderr')
    command.append('--batch')
    command.append('--no-sandbox')
    command.append('--minimize')

subprocess.call(command)
