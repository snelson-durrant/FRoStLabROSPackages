#!/bin/bash

sudo rm /etc/netplan/*
sudo cp ~/networkconfig/10-inhand-config.yaml /etc/netplan

sudo netplan apply
sudo netplan get
