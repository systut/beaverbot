# Confirm udev attributes for sensors

```
udevadm info --query=all --name=/dev/video0 | grep 'ID\|SUBSYSTEM'
```