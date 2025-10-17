# pke-starling-controllers
If you wanna use this you need to install `epicallypowerful` and my own little side project `starling`. It's basically ROS without ROS, and it has threadbare documentation right now so good luck :).

```console
# clone epicallypowerful
cd epicallypowerful
pip install -e .
pip install git+https://github.com/siddn/starling.git
```

If you want to use the services files in systemd, you'll need to make sure the python executable is correct, the script directories are correct, and that there are no passwords required for sudo commands.