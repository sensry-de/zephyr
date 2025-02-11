# Copyright (c) 2025 Sensry GmbH
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing Ganymed devices with serial loader tool.'''

import os
import sys
from os import path

from runners.core import RunnerCaps, ZephyrBinaryRunner


class GanymedBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for ganymed console loader.'''

    def __init__(self, cfg, device, reset=False, baud=1000000,
                 console='console', bootloader_bin=None):
        super().__init__(cfg)
        self.elf = cfg.elf_file
        self.app_bin = cfg.bin_file
        self.reset = bool(reset)
        self.device = device
        self.baud = baud
        self.console = console
        self.bootloader_bin = bootloader_bin

    @classmethod
    def name(cls):
        return 'ganymed'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, erase=True, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        # Required
        parser.add_argument('--console_path', required=True,
                            help='path to ganymed console')
        # Optional
        parser.add_argument('--baud-rate', default='1000000',
                            help='serial baud rate, default 1000000')
        parser.add_argument(
            '--console-tool',
            help='''if given, complete path to ganymed console. default is to search for
            it in zephyrproject/modules/tools/console_py/console.py''')
        parser.add_argument('--flash-coreguard',
                            help='coreguard image to flash')

        parser.set_defaults(reset=True)

    @classmethod
    def do_create(cls, cfg, args):
        if args.console_tool:
            console = args.console_tool
        else:
            console = path.join(args.console_path, 'tools', 'console_py',
                               'console.py')

        return GanymedBinaryRunner(
            cfg, args.device, reset=args.reset,
            baud=args.baud_rate, bootloader_bin=args.flash_bootloader)

    def do_run(self, command, **kwargs):
        self.require(self.console)

        # Add Python interpreter
        cmd_flash = [sys.executable, self.console]

        if self.device is not None:
            cmd_flash.extend([self.device])

        cmd_flash.extend([self.baud])

        if self.bootloader_bin:
            # add the coreguard binary file
        else:
            cmd_flash.extend([self.app_address, self.app_bin])

        self.logger.info(f"Flashing ganymed chip on {self.device} ({self.baud}bps)")
        # self.check_call(cmd_flash)
