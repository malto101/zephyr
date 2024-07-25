import os
import subprocess
from west.commands import WestCommand
from west import log

class RemoteFlash(WestCommand):

    def __init__(self):
        super().__init__(
            'remoteflash',
            'flash image to remote target',
            '''This command copies the Zephyr image to a remote device,
            configures the remoteproc module, and starts the core.'''
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description)
        
        parser.add_argument('--user', required=True,
                            help='The username for SSH connection')
        parser.add_argument('--remote', required=True,
                            help='The remote host address or name')
        parser.add_argument('--password', required=True,
                            help='The SSH password for the remote host')
        return parser

    def do_run(self, args, unknown_args):
        user = args.user
        remote = args.remote
        password = args.password

        log.banner('Copying Zephyr image to remote device...')
        copy_command = f'sshpass -p "{password}" scp build/zephyr/zephyr.elf {user}@{remote}:~/'
        self.run_command(copy_command)
        log.banner('Copy Successful')

        log.banner('Flashing Zephyr image to remote device...')
        flash_command = f'sshpass -p "{password}" ssh -tt -o StrictHostKeyChecking=no {user}@{remote} "echo {password} | sudo -S ./flash_script.sh"'
        self.run_command(flash_command)
        log.banner('Flashing Successful')

    def run_command(self, command):
        try:
            result = subprocess.run(command, shell=True, check=True, text=True, 
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            log.inf(result.stdout)
            if result.stderr:
                log.wrn(result.stderr)
        except subprocess.CalledProcessError as e:
            log.err(f'Command failed: {command}\nError: {e}')
            raise e