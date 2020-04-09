Troubleshooting
===============

Unable to contact ROS master at 192.168.100.1:11311
---------------------------------------------------

This means that the two computers in QT are not talking correctly because they did not boot in the right order.
To fix this, restart QT by pressing down QT's power button for a second or two.
QT should slump forward and the face screen should go off.
If the face screen doesn't go off, power QT on and repeat the process.
I haven't ever had to do this more than twice, as by the second time they are synced.

To test if the computers are talking correctly, you should be enter the following command and see a list of outputs.  If you get an error, restart QT.

::

    rostopic list

The tablet doesn't connect
--------------------------

This can be two things in my experience:

1. The tablet is on the wrong wifi network
2. The interaction isn't running (most commonly Fitbit credential errors)


Tablet is connecting to the wrong wifi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is easiest to check that the tablet is on the correct wifi.
When QT turns off, it turns off the wireless network that it is hosting.
When this happens, most tablets will auto-join other networks that they have been connected to.
What makes this worse is that web browsers often rememeber (cache) websites to make them quicker to load, which makes it look like you're connecting to the website hosted by QT, but you're not.
To fix this, go into the tablet's settings and turn off autojoin for all wireless networks except for QT (or just the networks you expect the tablet to see while the interaction is running).
Once this is set, you should be able to connect to and prompt the interaction on QT, including after QT restarts (if you've changed the container's restart policy).

Interaction isn't running
^^^^^^^^^^^^^^^^^^^^^^^^^

There can be a few reasons for this.
The most likely is that the Fitbit or Amazon Web Services credentials entered aren't working.
To see what it is for sure, turn on QT, open a terminal, go into the setup directory, make sure all docker containers are closed, and run a debug docker container::

    cd ~/abm-setup/docker
    docker-compose down
    bash docker.sh debug

This will open a new terminal that is the Docker container running on QT.
In this container, let's see what happens if we start the interaction::

    roslaunch abm_interaction qt_abm_interaction.launch

While it runs, be on the lookout for any red text.
If you see something about Fitbit credentials, you'll need to redo the Fitbit credentials.
If you see something about Amazon Web Services (AWS uploader, Polly, text-to-speech (tts)) you'll need to redo the Amazon Web Services Credentials.

To redo the Fitbit credentials, delete the current Fitbit credentials.  
From the terminal that opened up when you ran :code:`bash docker.sh debug`, enter the following and then run the setup script, again (below).

::

    rm /root/state/fitbit_credentials.yaml

If you need to redo the AWS credentials, just run the setup script.
You can do this from the terminal that opened when you ran :code:`bash docker.sh debug`::

    roslaunch abm_interaction setup_abm_interaction.launch

You can then run the interaction from the terminal that opened when you ran :code:`bash docker.sh debug`::

    roslaunch abm_interaction qt_abm_interaction.launch

If you are still getting an error here, check that you have access to the AWS bucket with the account credentials you've supplied (e.g., if the bucket is owned by the account you've entered in the AWS credentials).

If you suspect it is not, in the terminal that is on QT (not the Docker one opened with :code:`bash docker.sh debug`), change the AWS bucket specified at the end of the Dockerfile to a valid bucket owned by the AWS account you're using.  To open the Dockerfile, use the following command and go to the last line of the file::

    nano ~/abm-setup/docker/Dockerfile

.. note ::
    If neither of these fixed your error, let Audrow know and he will try to guide you through it and then fix the problem or add it to this document.

QT's head computer doesn't have internet
----------------------------------------

If you can SSH into QT's head computer, but are unable to clone a repository or make an update, check if you have internet.  The following command should show you that messages are being sent and responses are received, it just hangs there, you are not connected to the internet::

    ping google.com

At the moment, just restart QT and hopefully the problem will be fixed.
LuxAI has recognized this problem and given me steps to fix it but I haven't tested them.

If you would like to try, here is their email

    In our older setup of RPI/NUC network (same as your QT) we had enabled port forwarding (8080 -> 80) on both machine to facilitate accessing the QTrobot wev config. In some cases and time-to-time (also depending on the router) this causes problem for RPI to reach the internet properly.


    In this FAQ we explained it how to disable it: https://docs.luxai.com/FAQ/Network/

    Some more comments on this issue : https://github.com/luxai-qtrobot/QA/issues/3


    However all you need to do (as I imagine this can be the cause of the problem) is to disable the port forwarding on both machines QTPC and QTRP.

    For QTRP (RPI), just follow the simple instruction in the FAQ link and comment the corresponding line in :code`start_qt_routes.sh`.
    Double check that your :code:`/etc/network/interfaces` on RPI has the following config::

        auto lo eth0
        iface lo inet loopback
        auto eth0
        iface eth0 inet static
        address 192.168.100.1
        netmask 255.255.255.0
        gateway 192.168.100.2
        dns-nameservers 192.168.100.2 8.8.8.8

    Regarding the QTPC, you can completely disable/remove  the ‘start_qt_routes.sh’.