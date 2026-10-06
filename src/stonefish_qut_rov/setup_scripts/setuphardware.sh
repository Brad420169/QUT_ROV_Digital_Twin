#!/usr/bin/env bash
set -eo pipefail
if [[ "${1:-}" == --help ]]; then
    echo "Usage: bash src/stonefish_qut_rov/setup_scripts/setuphardware.sh [ETHERNET_INTERFACE | --network [ETHERNET_INTERFACE] | --camera]"
    exit 0
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
    echo "Real ROV requires Ubuntu 24.04 Desktop." >&2
    exit 1
fi
if [[ "$EUID" == 0 ]]; then
    echo "Run as your desktop user, without sudo. Administrator access is requested when needed." >&2
    exit 1
fi

setup_network() {
    interface="${1:-}"
    if [[ -z "$interface" ]]; then
        adapters=()
        while IFS=: read -r device kind; do
            [[ "$kind" != ethernet ]] || adapters+=("$device")
        done < <(nmcli -t -f DEVICE,TYPE device status)
        if [[ ${#adapters[@]} == 1 ]]; then
            interface="${adapters[0]}"
            printf 'Using the only Ethernet adapter: %s\n' "$interface"
        elif [[ ${#adapters[@]} == 0 ]]; then
            echo "No Ethernet adapter found. Plug in the tether adapter and rerun src/stonefish_qut_rov/setup_scripts/setuphardware.sh."
        else
            echo "Choose the Ethernet adapter connected to the ROV (leave internet on Wi-Fi):"
            for i in "${!adapters[@]}"; do
                printf '  %s) %s\n' "$((i+1))" "${adapters[i]}"
            done
            echo "  0) Set up Ethernet later"
            while true; do
                read -r -p "Adapter number: " choice
                if [[ "$choice" == 0 ]]; then break; fi
                if [[ "$choice" =~ ^[1-9][0-9]*$ && ${#choice} -le 3 ]] && (( choice <= ${#adapters[@]} )); then
                    interface="${adapters[choice-1]}"
                    break
                fi
                echo "Enter one of the numbers shown."
            done
        fi
    fi
    if [[ -n "$interface" ]]; then
        if [[ "$(nmcli -g GENERAL.TYPE device show "$interface")" != ethernet ]]; then
            echo "Not an Ethernet adapter: $interface" >&2
            exit 1
        fi
        if ! nmcli connection show Real_ROV_Tether >/dev/null 2>&1; then
            sudo nmcli connection add type ethernet con-name Real_ROV_Tether ifname "$interface" connection.autoconnect no
        fi
        sudo nmcli connection modify Real_ROV_Tether connection.interface-name "$interface" \
            ipv4.method manual ipv4.addresses '192.168.2.1/24,192.168.144.10/24' \
            ipv4.gateway '' ipv4.dns '' ipv4.never-default yes \
            ipv6.method link-local ipv6.never-default yes \
            connection.autoconnect yes connection.autoconnect-priority 100
        echo "Tether profile saved for Pixhawk and camera. It will activate when connected."
    fi
    [[ -n "$interface" ]]
}

setup_camera() {
    CAMERA=root@192.168.144.108
    SSH_OPTIONS=(-o ConnectTimeout=5 -o StrictHostKeyChecking=yes)
    CHECK='cat /sys/class/thermal/thermal_zone0/temp'
    if ssh "${SSH_OPTIONS[@]}" -o BatchMode=yes "$CAMERA" "$CHECK"; then
        echo "Camera SSH is already ready."
        return 0
    fi
    # Check reachability before asking the user to create or copy a key.
    if ! python3 - <<'CHECK_CAMERA'
import socket
try:
    with socket.create_connection(('192.168.144.108', 22), timeout=5):
        pass
except OSError as error:
    raise SystemExit(f'Camera SSH unavailable: {error}. Check camera power and tether.')
CHECK_CAMERA
    then
        exit 1
    fi
    key="$HOME/.ssh/id_ed25519"
    if [[ ! -f "$key" && -f "$HOME/.ssh/id_rsa" ]]; then
        key="$HOME/.ssh/id_rsa"
    fi
    if [[ ! -f "$key" ]]; then
        mkdir -p "$HOME/.ssh"
        chmod 700 "$HOME/.ssh"
        echo "Creating an SSH key. Choose a passphrase when prompted; unlock it in your desktop SSH agent before use."
        ssh-keygen -t ed25519 -f "$key" -C rov-camera
    fi
    if [[ ! -f "$key.pub" ]]; then
        echo "Public key missing: $key.pub. Restore it before continuing." >&2
        exit 1
    fi
    echo "Verify the camera host fingerprint when prompted, then enter the camera password. 123456 is the default password."
    ssh-copy-id -i "$key.pub" -o ConnectTimeout=5 -o StrictHostKeyChecking=ask "$CAMERA"
    if [[ -n "${SSH_AUTH_SOCK:-}" ]]; then
        if ! ssh "${SSH_OPTIONS[@]}" -o BatchMode=yes "$CAMERA" "$CHECK"; then
            echo "Unlocking the camera key in your desktop SSH agent."
            ssh-add "$key"
        fi
    fi
    if ! ssh "${SSH_OPTIONS[@]}" -o BatchMode=yes "$CAMERA" "$CHECK"; then
        printf 'Camera login is not ready for unattended use. If your key has a passphrase, run:\n  ssh-add "%s"\nThen rerun: bash src/stonefish_qut_rov/setup_scripts/setuphardware.sh --camera\n' "$key" >&2
        exit 1
    fi
    echo "Camera temperature and gimbal SSH access are ready."
}

case "${1:-}" in
    --network)
        shift
        if [[ $# -gt 1 ]]; then echo "Provide at most one Ethernet interface." >&2; exit 1; fi
        setup_network "$@"
        nmcli --wait 15 connection up Real_ROV_Tether
        exit 0
        ;;
    --camera)
        if [[ $# != 1 ]]; then echo "Usage: bash src/stonefish_qut_rov/setup_scripts/setuphardware.sh --camera" >&2; exit 1; fi
        setup_camera
        exit 0
        ;;
    --*) echo "Unknown option. Run bash src/stonefish_qut_rov/setup_scripts/setuphardware.sh --help." >&2; exit 1 ;;
esac
if [[ $# -gt 1 ]]; then
    echo "Provide at most one Ethernet interface, or use --help." >&2
    exit 1
fi
printf '\nSetting up Real ROV hardware. Power the ROV and camera and connect the tether. Keep internet on Wi-Fi.\n'
setup_network "$@"
nmcli --wait 15 connection up Real_ROV_Tether
setup_camera
echo "Hardware setup complete. Open ROV from the desktop or applications menu."
echo "No vehicle control was started."
