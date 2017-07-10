If you received the error messages like "Unable to receive message", try to run command "ldd libDefaultEnumeratorImpl.so" under "/opt/softkinetic/DepthSenseSDK/bin/Plugins".
If you get libudev.so.0 => not found message, please install libudev.
If the libudev.so.1 is located in your laptop, please run "sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1 /lib/libudev.so.0"

