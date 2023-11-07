#!/bin/bash

sudo rm /etc/netplan/*
sudo cp ~/networkconfig/20-frostUUV-config.yaml /etc/netplan

sudo netplan apply
sudo netplan get
