#!/bin/bash

# Script to mount HCRLAB flash drive for rosbag storage

FLASH_DRIVE_LABEL="HCRLAB"
MOUNT_POINT="/media/HCRLAB"

echo "Looking for flash drive with label: $FLASH_DRIVE_LABEL"

# Check if already mounted
if [ -d "$MOUNT_POINT" ]; then
    echo "Flash drive already mounted at $MOUNT_POINT"
    ls -la "$MOUNT_POINT"
    exit 0
fi

# Create mount point if it doesn't exist
sudo mkdir -p "$MOUNT_POINT"

# Find the flash drive by label
DEVICE=$(sudo blkid | grep "$FLASH_DRIVE_LABEL" | cut -d: -f1)

if [ -z "$DEVICE" ]; then
    echo "Flash drive with label '$FLASH_DRIVE_LABEL' not found."
    echo "Available devices:"
    sudo blkid
    exit 1
fi

echo "Found flash drive: $DEVICE"

# Mount the flash drive
echo "Mounting flash drive to $MOUNT_POINT..."
sudo mount "$DEVICE" "$MOUNT_POINT"

if [ $? -eq 0 ]; then
    echo "Flash drive mounted successfully!"
    echo "Contents:"
    ls -la "$MOUNT_POINT"
    
    # Create rosbags directory if it doesn't exist
    if [ ! -d "$MOUNT_POINT/rosbags" ]; then
        echo "Creating rosbags directory..."
        mkdir -p "$MOUNT_POINT/rosbags"
    fi
    
    echo "Flash drive ready for rosbag storage!"
else
    echo "Failed to mount flash drive"
    exit 1
fi 