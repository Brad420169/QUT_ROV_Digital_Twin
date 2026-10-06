#!/usr/bin/env python3
"""Create the shared ROV desktop/application shortcut for this checkout."""
from pathlib import Path
import os
import subprocess
root = Path(__file__).resolve().parent.parent
desktop = Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
applications = Path(os.environ.get('XDG_DATA_HOME') or Path.home() / '.local/share') / 'applications'
# Desktop Entry quoting has two layers: general string escapes, then Exec quoting.
script = str(root / 'ros_nodes/common/launch_rov_gui.sh')
quoted = '"' + ''.join(('\\' + c if c in '\\"`$' else c) for c in script) + '"'
quoted = quoted.replace('\\', '\\\\').replace('%', '%%')
icon = str(root / 'icons/rov_icon.png').replace('\\', '\\\\')
entry = ('[Desktop Entry]\nVersion=1.0\nType=Application\nName=ROV\n'
         'Comment=Open the QUT real ROV and digital twin launcher\nExec=/bin/bash ' + quoted + '\n'
         'Icon=' + icon + '\nTerminal=true\nCategories=Science;\n')
for directory in (desktop, applications):
    directory.mkdir(parents=True, exist_ok=True)
    shortcut = directory / 'QUT_ROV.desktop'
    shortcut.write_text(entry)
    shortcut.chmod(0o755)
    print('Created ' + str(shortcut))
shortcut = desktop / 'QUT_ROV.desktop'
if subprocess.run(['gio', 'set', str(shortcut), 'metadata::trusted', 'true']).returncode:
    print('If needed, right-click the desktop icon and select Allow Launching. The applications-menu entry also works.')
