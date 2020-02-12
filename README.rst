README
======

*Note that this is actively being written*

Instructions to setup a QT robot for the ABM SBIR study.

.. contents::

Instructions
------------

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

