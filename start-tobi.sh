#!/bin/bash

export RSB_TRANSPORT_SOCKET_ENABLED=0

export RSB_PLUGINS_CPP_LOAD=rsbspread
export RSB_TRANSPORT_SPREAD_ENABLED=1
export RSB_TRANSPORT_SPREAD_PORT=4803
export RSB_TRANSPORT_SPREAD_HOST=192.168.102.9

rsb-sendcpp0.13 /tobi ./rsb-payload.txt