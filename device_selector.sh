# Made by CAM
flash_device(){
    local devices=("$@")
    if [ ${#devices[@]} -eq 0 ]; then
        echo "No ttyUSB devices found"
        exit 1
    fi

    echo "Select a device to flash:"

    #show a select menu
    select dev in "${devices[@]}"; do
        if [ -n "$dev" ]; then
            echo "You selected: $dev"
            idf.py build && idf.py -p $dev flash
            break
        else
            echo "Invalid choice. Try again."
        fi
    done
}

monitor_devices(){
    local devices=("$@")
    if [ ${#devices[@]} -eq 0 ]; then
        echo "No ttyUSB devices found"
        exit 1
    fi

    echo "Select a device to monitor:"
    #show a select menu
    select dev in "${devices[@]}"; do
        if [ -n "$dev" ]; then
            echo "You selected: $dev"
            idf.py -p $dev monitor
            break
        else
            echo "Invalid choice. Try again."
        fi
    done
}

flash_and_monitor(){
    local devices=("$@")
    if [ ${#devices[@]} -eq 0 ]; then
        echo "No ttyUSB devices found"
        exit 1
    fi

    echo "Select a device to flash and monitor:"
    #show a select menu
    select dev in "${devices[@]}"; do
        if [ -n "$dev" ]; then
            echo "You selected: $dev"
            idf.py build && idf.py -p $dev flash && idf.py -p $dev monitor
            break
        else
            echo "Invalid choice. Try again."
        fi
    done
}


echo "============ESP Flash and Monitor Tool============"

devices=($(ls /dev/ttyUSB* 2>/dev/null))

#ensure that there are devices
if [ ${#devices[@]} -eq 0 ]; then
    echo "No ttyUSB devices found"
    exit 1
fi



PS3="Select the option: "
select opt in Flash Monitor FlashMonitor Quit; do
    case $opt in
        Flash)
            echo "You selected to Flash Device!"
            flash_device "${devices[@]}"
            ;;
        Monitor)
            echo "You selected to Monitor!"
            monitor_devices "${devices[@]}"
            ;;
        FlashMonitor)
            flash_and_monitor "${devices[@]}"
            ;;
        Quit)
            echo "Goodbye!!"
            break
            ;;
        *)
            echo "Invalid Option!"
            ;;
    esac
done
