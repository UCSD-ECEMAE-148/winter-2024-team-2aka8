If your vesc cuts out at speeds above ~0.25 of the robot's maximum speed, it is probably because the erpm limits are set very low by default

To fix, open the vesc tool, and change the fields "Sensorless ERPM," and Hall Interpolation ERPM to 50000. They are under the Hall sensors tab in the FOC section.

Also verify that the erpm in the general-rpm section of the vesc tool is set to at least 50000.

The motor has a maximum theoretical erpm of 106560 (3600kv x 14.8v x 4poles/2), but it is risky setting the limit that high. The motor is also only rated for 3s batteries powering it, but we are running 4s batteries which is pushing it a little harder so it is good to be a little conservative with the erpm limit.

You may want to increase the current limits as well to get faster acceleration. The vesc is limited to 50A continuous and 150A instantaneous current.

Motor product page:
https://www.hobbywingdirect.com/products/xerun-3660-g2-sensored-motor (we are using the 3600 kv variant, or my robocar was at least)

VESC product page:
https://flipsky.net/products/mini-fsesc4-20-50a-base-on-vesc-widely-used-in-eskateboard-escooter-ebike
