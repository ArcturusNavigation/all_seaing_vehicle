"""
Execution targets for the control panel: where commands run.

- SSHTarget: the Jetson (or any machine reachable with key-based SSH), via one persistent
  paramiko connection.
- LocalTarget: the laptop the panel runs on (e.g. for simulation).

Both expose the same interface:
    run(cmd, timeout, stdin=None) -> (returncode, stdout, stderr)   blocking, with ROS sourced
    start(cmd, log_path) -> pid        detached in its own process group (survives the panel)
    alive(pid) -> bool
    signal(pid, sig)                   to the whole process group
    read_file(path, offset, max_bytes) -> (bytes, new_offset)
    write_file(path, text)
    download(remote_path, local_path, progress=None)   progress(done_bytes, total_bytes)
Every command runs in `bash -lc` after the target's `setup` lines (e.g. sourcing ROS and
the workspace), configured in config/targets.yaml.
"""

import os
import shlex
import subprocess
import threading

import paramiko


class Target:
    def __init__(self, name, cfg):
        self.name = name
        self.label = cfg.get("label", name)
        self.setup = cfg.get("setup", [])
        self.paths = cfg.get("paths", {})  # repo, workspace, logs, bags, panel (process logs)

    def path(self, key):
        return self.paths.get(key, "")

    def wrap(self, cmd):
        """Full bash command: setup lines, then cmd."""
        return "bash -lc " + shlex.quote("; ".join(self.setup + [cmd]))

    def start(self, cmd, log_path):
        """Start cmd detached (own session/process group), output to log_path. Returns the PID."""
        log_dir = os.path.dirname(log_path)
        # `&` must apply to setsid alone (not to a `mkdir && setsid` list): a backgrounded list runs
        # in a subshell that keeps the SSH channel open until the process exits
        launcher = (f"mkdir -p {shlex.quote(log_dir)}; "
                    f"setsid nohup {self.wrap(cmd)} > {shlex.quote(log_path)} 2>&1 < /dev/null & echo $!")
        rc, out, err = self._raw(launcher, timeout=15)
        if rc != 0 or not out.strip().isdigit():
            raise RuntimeError(f"failed to start on {self.name}: {err or out}")
        return int(out.strip())

    def alive(self, pid):
        rc, _, _ = self._raw(f"kill -0 {int(pid)} 2>/dev/null", timeout=10)
        return rc == 0

    def signal(self, pid, sig="INT"):
        # negative PID: the whole process group started by setsid (launch + all its nodes)
        self._raw(f"kill -s {sig} -- -{int(pid)} 2>/dev/null || kill -s {sig} {int(pid)} 2>/dev/null", timeout=10)

    def read_file(self, path, offset=0, max_bytes=200_000):
        rc, out, _ = self._raw(f"tail -c +{int(offset) + 1} {shlex.quote(path)} 2>/dev/null | head -c {int(max_bytes)}",
                               timeout=15, binary=True)
        return out, offset + len(out)

    def write_file(self, path, text):
        rc, _, err = self._raw(f"cat > {shlex.quote(path)}", timeout=15, stdin=text)
        if rc != 0:
            raise RuntimeError(f"failed to write {path} on {self.name}: {err}")

    def run(self, cmd, timeout=60, stdin=None):
        return self._raw(self.wrap(cmd), timeout=timeout, stdin=stdin)

    def _raw(self, cmd, timeout, stdin=None, binary=False):
        raise NotImplementedError


class LocalTarget(Target):
    kind = "local"

    def path(self, key):
        """Configured path with ~ expanded (paths are shell-quoted in commands, which disables ~)."""
        return os.path.expanduser(super().path(key))

    def _raw(self, cmd, timeout, stdin=None, binary=False):
        try:
            p = subprocess.run(["bash", "-c", cmd], input=stdin.encode() if stdin else None,
                               capture_output=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return 124, b"" if binary else "", "timeout"
        out = p.stdout if binary else p.stdout.decode(errors="replace")
        return p.returncode, out, p.stderr.decode(errors="replace")

    def download(self, remote_path, local_path, progress=None):
        with open(os.path.expanduser(remote_path), "rb") as src, open(local_path, "wb") as dst:
            _copy_with_progress(src, dst, os.path.getsize(os.path.expanduser(remote_path)), progress)


def _copy_with_progress(src, dst, total, progress, chunk=4 << 20):
    done = 0
    while True:
        buf = src.read(chunk)
        if not buf:
            break
        dst.write(buf)
        done += len(buf)
        if progress:
            progress(done, total)


class SSHTarget(Target):
    kind = "ssh"

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.host = cfg["host"]
        self.user = cfg.get("user")
        self._client = None
        self._lock = threading.Lock()

    def _connect(self):
        if self._client is not None and self._client.get_transport() and self._client.get_transport().is_active():
            return self._client
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        client.connect(self.host, username=self.user, timeout=5, banner_timeout=5, auth_timeout=5)
        client.get_transport().set_keepalive(10)
        self._client = client
        return client

    def connected(self):
        try:
            with self._lock:
                self._connect()
            return True
        except Exception:
            return False

    def _raw(self, cmd, timeout, stdin=None, binary=False):
        try:
            with self._lock:
                client = self._connect()
            chan_in, chan_out, chan_err = client.exec_command(cmd, timeout=timeout)
            if stdin is not None:
                chan_in.write(stdin)
                chan_in.channel.shutdown_write()
            out = chan_out.read()
            err = chan_err.read().decode(errors="replace")
            rc = chan_out.channel.recv_exit_status()
        except Exception as e:  # connection lost / timeout: reconnect next time
            self._client = None
            return 255, b"" if binary else "", f"ssh error: {e}"
        return rc, out if binary else out.decode(errors="replace"), err

    def download(self, remote_path, local_path, progress=None):
        """Copy a file from the target over SFTP (its own channel: commands keep working meanwhile)."""
        with self._lock:
            client = self._connect()
        sftp = client.open_sftp()
        try:
            sftp.get(remote_path, local_path, callback=progress)
        finally:
            sftp.close()


def make_targets(cfg):
    targets = {}
    for name, tcfg in cfg.items():
        targets[name] = SSHTarget(name, tcfg) if tcfg.get("kind") == "ssh" else LocalTarget(name, tcfg)
    return targets
