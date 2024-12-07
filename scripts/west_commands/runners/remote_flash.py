from runners.core import ZephyrBinaryRunner
import subprocess
import os
import yaml

# Mapping of board names to remote processor paths
BOARD_PATHS = {
    "beaglebone_ai64": "/dev/remoteproc/j7-main-r5f0_0",
    "kv260": "/sys/class/remoteproc/remoteproc0"
}


class RemoteProcFlashRunner(ZephyrBinaryRunner):
    def __init__(self, cfg, username, ipaddress, firmware_path, remoteproc_path):
        super().__init__(cfg)
        self.username = username
        self.ipaddress = ipaddress
        self.firmware_path = firmware_path
        self.remoteproc_path = remoteproc_path

    @classmethod
    def name(cls):
        return 'remote_flash'

    @classmethod
    def capabilities(cls):
        return {
            'commands': ['flash'],
            'flash_addr': False,
            'ram': False
        }

    @classmethod
    def create(cls, cfg, args):
        if len(args) != 3:
            raise ValueError(
                "Expected 3 arguments: username, ipaddress, firmware_path")

        username, ipaddress, firmware_path = args

        # Parse board information from build_info.yml
        build_info_path = os.path.join(cfg.build_dir, "build_info.yml")
        if not os.path.exists(build_info_path):
            raise FileNotFoundError(
                f"Build info file not found: {build_info_path}")

        with open(build_info_path, 'r') as f:
            build_info = yaml.safe_load(f)
            board_name = build_info.get('board', {}).get('name')
            if not board_name:
                raise ValueError("Board name not found in build_info.yml")

        # Map board to remoteproc path
        remoteproc_path = BOARD_PATHS.get(board_name)
        if not remoteproc_path:
            raise ValueError(f"Unknown board '{board_name}'. Supported boards: {
                             ', '.join(BOARD_PATHS.keys())}")

        return cls(cfg, username, ipaddress, firmware_path, remoteproc_path)

    def do_run(self, command, **kwargs):
        if command != 'flash':
            raise ValueError(f"Unsupported command: {command}")

        elf_path = os.path.join(self.cfg.build_dir, "zephyr", "zephyr.elf")

        # Step 1: Copy firmware to remote device
        scp_command = f"sudo scp {elf_path} {
            self.username}@{self.ipaddress}:{self.firmware_path}"
        self._run_command(scp_command, "Copying firmware")

        # Step 2: SSH into the target and stop the remote processor
        ssh_commands = [
            f"echo stop > {self.remoteproc_path}/state",
            f"echo {os.path.basename(elf_path)} > {
                self.remoteproc_path}/firmware",
            f"echo start > {self.remoteproc_path}/state"
        ]
        for ssh_command in ssh_commands:
            self._ssh_command(ssh_command, "Executing remote command")

    def _run_command(self, command, description):
        print(f"{description}: {command}")
        try:
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Command failed: {command}\nError: {e}") from e

    def _ssh_command(self, command, description):
        ssh_command = f"ssh {self.username}@{self.ipaddress} '{command}'"
        print(f"{description}: {ssh_command}")
        try:
            subprocess.run(ssh_command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"SSH Command failed: {ssh_command}\nError: {e}") from e
