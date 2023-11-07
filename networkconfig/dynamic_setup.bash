#!/bin/bash

sudo rm /etc/netplan/*
sudo cp ~/networkconfig/15-inhandDynamic-config.yaml /etc/netplan

sudo netplan apply
sudo netplan get
