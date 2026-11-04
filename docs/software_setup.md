# Software setup and installation

[Repository overview](../README.md) | [Documentation guide](README.md)

Start with [GitLab access](#access-nikhef-gitlab), then follow
[Native setup](#native-setup) to install the tools, download the course and
create its Python environment.

The native setup runs Python directly on your laptop and installs the lecture packages.

Before the lectures, complete this setup and the
[software-only checks](#native-simulations-and-notebooks). The [USB](#usb-setup) and
[radio checks](#team-radio-configuration) follow when your group has its hardware.

**Neither Docker nor Colima is needed to connect to or fly a drone.**

## Access Nikhef GitLab

Open the [course project](https://gitlab.nikhef.nl/graven/crazyflie-topical-lectures/).
If you reach the sign-in page:

1. Choose **Federated login**.
2. Select your home institution. Nikhef users select **Nikhef SSO**; external
   participants select their own institution and use its usual login details.
3. Complete any consent or account-creation prompts, then reopen the course link.

If your institution is missing, federated login fails, or the project remains
inaccessible after login, contact
[Nikhef Helpdesk](mailto:helpdesk@nikhef.nl) directly.
Follow [Nikhef's GitLab access instructions](https://kb.nikhef.nl/ct/Gitlab.html).

For installation questions and finding teammates, log in to
[Nikhef Mattermost](https://mattermost.nikhef.nl) via **GitLab**, then
**Federated login** if prompted. Click your profile picture (or initials), choose
**Profile**, and copy the value in the **Username** field. Add an `@` prefix to form your `@username`: for example,
`a.smith` becomes `@a.smith`. Submit it through the
[Indico form](https://indico.nikhef.nl/event/8196/surveys/13) so the organizers can
add you to the lecture channel.
For Mattermost login problems, contact Helpdesk as above.

## Native setup

Follow these steps:

1. Install any missing prerequisite tools for [macOS](#macos), [Linux](#linux)
   or [Windows](#windows). If they are already installed, skip their installation.
2. [Download the course](#download-the-course) to your laptop.
3. [Set up this project](#create-the-project-environment) in the cloned folder.
   This step is still needed when Git, Python and the other tools are installed.

**Python 3.12.x is the tested default.** Newer versions are allowed with a
warning but have not been validated with the pinned packages, Qt GUI, and USB
access; older versions are rejected.

Initial installation needs internet access and space for the Python environment
and downloaded packages. Installing a package manager, Linux libraries or Windows
USB drivers may require administrator access; arrange that in advance on managed
laptops. The installed simulations and supplied notebook then work offline.

### macOS

If Git/developer command-line tools are missing, install them first:

```bash
xcode-select --install
```

Choose either [Homebrew](https://brew.sh/) or
[MacPorts](https://www.macports.org/install.php).
Install the package manager first and follow its instructions to add it to
`PATH`. Both routes below include `libusb` for Crazyradio access.

**Homebrew:**

```bash
brew install python@3.12 libusb
"$(brew --prefix)/bin/python3.12" --version
```

**MacPorts:**

```bash
sudo port install python312 libusb
/opt/local/bin/python3.12 --version
```

See also the [Python macOS guide](https://docs.python.org/3.12/using/mac.html).

Next, [download the course](#download-the-course).

### Linux

Install Python 3.12 and its `venv` support using your distribution's packages;
see the [Python Unix guide](https://docs.python.org/3.12/using/unix.html).
Also install the system USB library and Qt/X11 prerequisites for cfclient.
For Ubuntu releases providing Python 3.12 (for example Ubuntu 24.04):

```bash
sudo apt-get update
sudo apt-get install git python3.12 python3.12-venv libusb-1.0-0 libxcb-xinerama0 libxcb-cursor0
```

Other distributions use different package names; follow the
[cfclient Linux prerequisites](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/).
If your distribution supplies a newer Python, use the explicit `PYTHON`
override below rather than replacing the system Python. The default command
looks specifically for `python3.12`, not `python3`.

Next, [download the course](#download-the-course).

### Windows

Install the [Python Install Manager](https://docs.python.org/3/using/windows.html),
then run in PowerShell:

```powershell
pymanager install 3.12
py -3.12 --version
```

The project setup script defaults to `py -3.12`; installing Python alone does not install the
Crazyradio driver. Complete the Windows USB steps below separately.
`pymanager` explicitly selects the install manager, even when an older `py`
launcher is also installed. Open a new PowerShell window if the command is not
found immediately after installing the manager.

Next, [download the course](#download-the-course).

### Download the course

[Clone the course](course_git.md#clone-the-course) into a writable folder.
If you already cloned it, open a terminal in that project
folder instead.



Run the following commands from the project root. Commands that need another
folder explicitly include a step such as `cd code`.

### Create the project environment

Run this step from the project root even if all prerequisite tools were already
installed. The setup script creates a project-specific `.venv` folder containing
the Python environment and lecture packages, or updates it if it already exists.
It installs `requirements-hardware.txt` and `requirements-simulation.txt`, and
runs the environment validator.
An existing Conda environment, such as `(base)`, is separate from this project's
`.venv` and does not replace this step.

Choose one setup command below. The platform commands use Python 3.12. If you
are using a newer Python, use the alternative command instead:

- macOS/Linux: `PYTHON=/path/to/python3.14 bash scripts/setup-unix.sh`
- Windows PowerShell: set `$env:PYTHON = 'C:\path\to\python.exe'`, then run
  `.\scripts\setup-windows.ps1`. If script execution is blocked, first run
  `Set-ExecutionPolicy -Scope Process Bypass`.

Use a fresh course clone when changing interpreter versions
so an existing `.venv` is not reused with another Python.

**macOS with Homebrew:**

```bash
PYTHON="$(brew --prefix)/bin/python3.12" bash scripts/setup-unix.sh
```

**macOS with MacPorts:**

```bash
PYTHON=/opt/local/bin/python3.12 bash scripts/setup-unix.sh
```

**Linux, or macOS with `python3.12` already on PATH:**

```bash
python3.12 --version
bash scripts/setup-unix.sh
```

**Windows PowerShell:**

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\setup-windows.ps1
```

Wait for setup to finish successfully before activation. If it fails, keep the
error output and resolve that error first. Updating the checkout with `git pull`
preserves an existing environment; rerun setup if the requirements change.

### Activate and check

After successful project setup, activate the environment in each new terminal:

```bash
# macOS/Linux
source .venv/bin/activate
```

```powershell
# Windows PowerShell
Set-ExecutionPolicy -Scope Process Bypass
.\.venv\Scripts\Activate.ps1
```

If activation reports `no such file or directory` or cannot find the script,
check that the terminal is in the project root. If `.venv` is missing there,
return to [Create the project environment](#create-the-project-environment).
Activation selects an existing environment; it does not create one.

The PowerShell policy change lasts only for that terminal; managed policies may
prevent it. Activation affects only the current terminal on all platforms.

With the environment active, run these software-only checks:

```bash
python -c "import sys; print(sys.executable)"
python scripts/validate_environment.py
cfclient
```

These commands check three different things:

1. The first prints the Python interpreter path. It should be inside this
   project's `.venv` folder.
2. `python scripts/validate_environment.py` checks package imports and versions,
   and ends with `Environment ready.` if the checks pass.
3. Running `cfclient` opens its window and checks that the graphical interface
   starts correctly. Leave it disconnected from the drone, then close the window
   before continuing.

If local policy prevents PowerShell activation, the environment's executables
can be run directly: `.\.venv\Scripts\python.exe` and
`.\.venv\Scripts\cfclient.exe`. If policy also prevents running the setup script,
arrange a permitted installation before arrival or share a working group laptop.

## Native simulations and notebooks

From the project root, with `.venv` active, run:

```bash
python code/day1/testing/test_module.py
python -m ipykernel install --sys-prefix --name crazyflie-lectures --display-name "Crazyflie lectures (.venv)"
jupyter lab
```

**Expected:** the simulator tests report that all tests passed.

Open Jupyter's printed URL and, in its file browser, open
**`code/day1/00_jupyter_example.ipynb`**. Start Jupyter from the project root as
shown above so the notebooks can also open links into `docs/`.
Select the **Crazyflie lectures (.venv)** kernel. Check it by running this in
a notebook cell:

```python
import sys
print(sys.executable)
```

The path should point into this project's `.venv`. If it differs from the
terminal check, select **Crazyflie lectures (.venv)** or close Jupyter and restart
it from the activated terminal. The kernel-registration command above stores
this kernel inside the project's environment. Run the primer's cells from top
to bottom and check that its plots appear below the cells. The remaining
[Day 1 notebooks](../code/day1/README.md) cover the scheduled controller experiments;
the interactive keyboard simulation uses a separate Qt plot window and is checked
during the Day 1 software session.

Days 2 and 3 use the same environment and kernel. When you reach those sessions,
open [day2/01_tracking.ipynb](../code/day2/01_tracking.ipynb) or
[day3/02_trajectory.ipynb](../code/day3/02_trajectory.ipynb) in Jupyter for the scheduled
filter experiments, trajectory planning and recording analysis. On Day 2, continue
with [02 calibration](../code/day2/02_calibration.ipynb) and
[03 height and velocity](../code/day2/03_height_velocity.ipynb), running each
notebook's setup in its own kernel. Results go under `code/day2/data/<run_id>/`.
Their default paths
work offline; live recordings and measured courses can be selected during the
practicals. There is no extra software installation or pre-lecture assignment.

**Pre-lecture setup is complete** when the validator passes, cfclient opens,
the simulation/test checks pass and the notebook produces its plots using the
intended interpreter. Keep any error output if a check fails and compare it
within your group.

## USB setup

Each group needs one radio and its assigned drone. Keep the drone's motors
stopped during connection checks.

### Windows

1. Download [Zadig](https://zadig.akeo.ie/) and connect the Crazyradio.
2. Run Zadig as administrator. If the radio is not listed, select
   **Options -> List All Devices**.
3. Select **Crazyradio** or **Crazyradio 2.0** in the device drop-down menu at the
   top of the window. Check the device name carefully before changing its driver.
4. Use the driver selector to the right of the green arrow to select
   **libusb-win32**, then click **Install Driver** or **Replace Driver**.
   If the radio is unreliable with this driver, Bitcraze recommends trying
   **libusbK** instead.
5. Unplug and reconnect the radio, then run the radio check again.

See Bitcraze's
[Windows USB-driver instructions](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/).

### Linux

Add your account to the `plugdev` group. Log out and back in after these commands:

```bash
sudo groupadd -f plugdev
sudo usermod -a -G plugdev "$USER"
```

Create the rule file with the following complete command. It writes this one
file with administrator permissions:

```bash
sudo tee /etc/udev/rules.d/99-bitcraze.rules >/dev/null <<'EOF'
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Crazyradio bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie connected directly over USB
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
```

Reload the rules, then unplug and reconnect the radio:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

See Bitcraze's
[Linux USB-permission instructions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/).

### macOS

The native setup commands above already install `libusb`.

Connect the Crazyradio directly to the Mac where possible. On Apple-silicon
Mac laptops, unlock the Mac and click **Allow** if an accessory prompt appears.
If you denied it, unplug and reconnect the radio while the Mac is unlocked
to request approval again. To change the policy, open **System Settings ->
Privacy & Security -> Accessories -> Allow accessories to connect**; some
macOS versions show the control directly under Privacy & Security.
This setting does not apply to every Mac: its absence on Intel Macs or desktop
Macs is not evidence of a USB-permission problem. Managed laptops may have an
administrator-controlled policy. See [Apple's accessory-access guide](https://support.apple.com/102282).
With a non-standard Homebrew prefix,
follow Bitcraze's
[macOS library-path note](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/).

## Team radio configuration

Organizers prepare each drone's radio settings and sticker label. Use that
`LABEL` in the connection notebook.

The drones are programmed before the lectures and identified by their sticker
labels. Start with the [Day 1 connection notebook](../code/day1/drone_01_connect.ipynb),
entering your drone's `LABEL`. Its discovery, status and LED checks establish
which drone you are using. Days 2 and 3 use the same label convention in their
**Connect** cells. Keep that selection for recording, surveying and flight;
the notebooks supply the discovered [radio URI](glossary.md#radio-uri) to the
tools that need it.

After [USB setup](#usb-setup), plug in the radio and run:

```bash
python scripts/validate_environment.py --radio
```

**Expected:** the validator lists the radio's serial number. Close cfclient
before using a connection notebook. The validator detects the USB dongle;
the notebook checks the link and drone identity. For later cfclient checks,
reuse that discovered address as described in
[B2](day_1_baseline_checks.md#b2-cfclient-connection-firmware-and-controller-identity).

## Troubleshooting

| Symptom | First checks |
| --- | --- |
| Python missing, wrong version or wrong interpreter | Use the OS-specific Python command above. After activation, `python -c "import sys; print(sys.executable)"` should show the project's `.venv`. |
| `pymanager` missing or `py install` tries to open a file named `install` | Install the linked Python Install Manager, reopen PowerShell and use `pymanager install 3.12`; an older `py` launcher may still be present. |
| pip fails or reports no matching distribution | Keep the first error, check internet access and Python version/architecture, and retry with the tested Python 3.12 in a fresh project copy. Preserve the pinned requirements. |
| cfclient fails with a Qt/xcb plugin error | On Linux, install the Qt libraries listed above and use a graphical desktop session. Retain the full terminal error; an import check alone cannot verify the GUI. |
| macOS reports `NoBackendError` or cannot find libusb | Check that Python and libusb come from the chosen package manager. For MacPorts, try `export DYLD_LIBRARY_PATH="/opt/local/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"` in the same terminal, then retry. For Homebrew, use the linked library-path note. |
| Radio absent or permission denied | Check the OS-specific driver/udev steps, reconnect the radio, try a direct USB port and close other radio clients. On Linux, confirm the new group membership after logging in again. |
| Simulation/notebook output already exists | Choose a new CLI output directory or change the notebook's `run_id`; keep previous results for comparison. |

If a problem remains, keep the command, working directory, OS, Python version
and full error output. The group can continue on a working laptop while retaining
the unresolved check for later investigation.

## Optional containers

For the alternative Docker-based simulation and notebook environment, follow
the [container setup guide](container_setup.md).
