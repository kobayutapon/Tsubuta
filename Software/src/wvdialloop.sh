#!/bin/bash
LOG='/var/log/wvdial'

(
while : ; do
wvdial 2>&1
sleep 15
done
) 2>&1 > ${LOG} < /dev/null &

