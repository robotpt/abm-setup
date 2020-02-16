######
README
######

.. warning::

    This is actively being written*

Instructions to setup a QT robot for the ABM SBIR study.

.. contents::

*****
Setup
*****

What you'll need
================

* A monitor
* A mouse and keyboard
* Wireless internet
* An `Amazon Web Services <https://aws.amazon.com/>`_ account
* An account with `Fitbit <https://www.fitbit.com/setup/>`_

QT has two ports: one USB-C and one USB-A.  You'll need to use the USB-C port for display and figure out how to control a mouse and keyboard.  I suggest a USB-C hub that has a display port that you can use (USB-C or HDMI, for example) and has USB-A ports for your mouse and keyboard.  Alternatively, you can use a USB-A hub to connect your mouse and keyboard to QT.

.. note::

    If you have trouble using your mouse or keyboard through a USB-C port, try flipping the USB-C input going into QT.  In theory, USB-C should go both ways, but in practice, sometimes not.

Basics
======

Turning QT on and off
---------------------

To turn on QT, just plug in power to QT and it will boot.

To turn off QT, there are two options:

    1. Press the button on the backside of QT, near its feet.

    2. Login to the head computer (see below) and do :code:`sudo shutdown`.

Note that if you do :code:`sudo shutdown` on the body computer, you only turn off the body computer---the head computer is still on.

.. note::

    If you unplug QT to restart QT, it may mess up the boot timing of the two computers.  Probably one of them takes longer because it boots in recovery mode.  This screws up how the head and body computer network.  You will not be able to connect to the head computer from the body computer.  If this occurs, simple restart QT by pushing the button on its backside.

Accessing QT's body computer
----------------------------

When you connect a monitor to QT and turn QT on, you will start on QT's body computer.

Accessing QT's head computer
----------------------------

On QT's body computer (the NUC)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0. Clone this repository onto QT's body computer

1. Setup Docker

    a. Install Docker::

        curl -fsSL https://get.docker.com -o get-docker.sh
        sh get-docker.sh

    b. Set Docker to run without `sudo`::

        sudo groupadd docker
        sudo gpasswd -a $USER docker
        newgrp docker

    c. Test that Docker is installed correctly and works without `sudo`::

        docker run hello-world

2. Setup Docker-compose:
   
    a. Install Docker-compose:: 

        sudo curl -L "https://github.com/docker/compose/releases/download/1.25.3/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        sudo chmod +x /usr/local/bin/docker-compose

    b. Check that docker compose is installed correctly::
        
        docker-compose version

3. Run the docker container

