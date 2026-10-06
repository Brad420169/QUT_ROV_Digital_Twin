#!/usr/bin/env python3
"""Offline setup checks: no sudo, installation, network or vehicle connection."""
import ast
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parent


class SetupChecks(unittest.TestCase):
    def test_shell_syntax_and_help(self):
        for script in SCRIPTS.glob('*.sh'):
            subprocess.run(['bash', '-n', str(script)], check=True)
            result = subprocess.run(['bash', str(script), '--help'], check=True,
                                    capture_output=True, text=True)
            self.assertIn(str(script.relative_to(SCRIPTS.parents[2])), result.stdout)
        installer = (SCRIPTS / 'install_dt.sh').read_text()
        ast.parse(installer.split("<<'PY'\n")[1].split('\nPY\n')[0])

    def test_dependency_checkout_preserves_existing_work(self):
        installer = (SCRIPTS / 'install_dt.sh').read_text()
        function = installer.split('checkout_dependency() {', 1)[1].split('\n}\n', 1)[0]
        function = 'set -e\ncheckout_dependency() {' + function + '\n}\ncheckout_dependency "$@"'
        with tempfile.TemporaryDirectory(prefix='rov git check ') as tmp:
            upstream = Path(tmp) / 'upstream'
            target = Path(tmp) / 'workspace with spaces/dependency'
            upstream.mkdir()
            def git(*args):
                return subprocess.check_output(['git', '-C', str(upstream), *args], text=True).strip()
            git('init', '-q')
            (upstream / 'source').write_text('original\n')
            git('add', 'source')
            git('-c', 'user.name=Setup Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'initial')
            revision = git('rev-parse', 'HEAD')
            def checkout(rev):
                return subprocess.run(['bash', '-c', function, 'test', str(upstream), str(target), rev],
                                      capture_output=True, text=True)
            self.assertEqual(checkout(revision).returncode, 0)
            self.assertEqual(checkout(revision).returncode, 0)
            self.assertNotEqual(checkout('0' * 40).returncode, 0)
            (target / 'source').write_text('my edits\n')
            self.assertNotEqual(checkout(revision).returncode, 0)
            self.assertEqual((target / 'source').read_text(), 'my edits\n')

    def test_desktop_shortcut_after_workspace_rename(self):
        with tempfile.TemporaryDirectory(prefix='rov desktop check ') as tmp:
            root = Path(tmp)
            package = root / 'ROV and DT ws/src/stonefish_qut_rov'
            scripts = package / 'setup_scripts'
            scripts.mkdir(parents=True)
            helper = scripts / 'create_shortcut.py'
            shutil.copyfile(SCRIPTS / helper.name, helper)
            desktop = root / 'Desktop'
            with patch.dict(os.environ, {'XDG_DATA_HOME': str(root / 'data')}), \
                 patch('subprocess.check_output', return_value=str(desktop) + '\n'), \
                 patch('subprocess.run') as run:
                run.return_value.returncode = 0
                runpy.run_path(str(helper), run_name='__main__')
            shortcut = desktop / 'QUT_ROV.desktop'
            entry = shortcut.read_text()
            self.assertIn(f'Exec=/bin/bash "{package}/ros_nodes/common/launch_rov_gui.sh"\n', entry)
            self.assertIn('Name=ROV\n', entry)
            self.assertIn(f'Icon={package}/icons/rov_icon.png\n', entry)
            self.assertTrue(os.access(shortcut, os.X_OK))
            self.assertEqual(entry, (root / 'data/applications/QUT_ROV.desktop').read_text())


if __name__ == '__main__':
    unittest.main()
