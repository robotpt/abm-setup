Setuping up QT
==============

.. |version| replace:: abm_pilot

What you'll need
----------------

* A monitor
* A mouse and keyboard
* Wireless internet
* An `Amazon Web Services <https://aws.amazon.com/>`_ account
* An account with `Fitbit <https://www.fitbit.com/setup/>`_
* A tablet for the user interface
* A Google account to get an app for the tablet

QT has two ports: one USB-C and one USB-A.  You'll need to use the USB-C port for display and figure out how to control a mouse and keyboard.  I suggest a USB-C hub that has a display port that you can use (USB-C or HDMI, for example) and has USB-A ports for your mouse and keyboard.  Alternatively, you can use a USB-A hub to connect your mouse and keyboard to QT.

.. note::

    If you have trouble using your mouse or keyboard through a USB-C port, try flipping the USB-C input going into QT.  In theory, USB-C should go both ways, but in practice, sometimes not.

Basics
------

Turning QT on and off
^^^^^^^^^^^^^^^^^^^^^

To turn on QT, just plug in power to QT and it will boot.

To turn off QT, there are two options:

    1. Press the button on the backside of QT, near its feet.

    2. Login to the head computer (see below) and do :code:`sudo shutdown`.

If you do :code:`sudo shutdown` on the body computer, you only turn off the body computer---the head computer is still on.

.. note::

    If you unplug QT to restart QT, it may mess up the boot timing of the two computers.  Probably one of them takes longer because it boots in recovery mode.  This screws up how the head and body computer network.  You will not be able to connect to the head computer from the body computer.  If this occurs, simple restart QT by pushing the button on its backside.

Accessing QT's body computer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When you connect a monitor to QT and turn QT on, you will start on QT's body computer.

Accessing QT's head computer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To setup the head, you must Secure-SHell into it (SSH) from QT's body computer.  To do this

0. Turn on QT.

1. Open a terminal.

2. Type the following and hit return::

    ssh qtrobot@192.168.100.1

Head
----


Turning off the default face
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0. If you haven't already, SSH into QT's head computer::

    ssh qtrobot@192.168.100.1

1. Update QT::

    cd ~/robot/packages/deb
    git pull
    sudo dpkg -i ros-kinetic-qt-robot-interface_1.1.8-0xenial_armhf.deb

.. note::

    If the :code:`git pull` step fails, the head computer might be having trouble with it its network.  You can check this with :code:`ping google.com`.  If there's nothing, there is a problem with the network.  To fix this, the best think we've found is to restart QT: :code:`sudo reboot`.

2. Edit a configuration file to turn off QT's default face:

    a. Open the configuration file::

        sudo nano /opt/ros/kinetic/share/qt_robot_interface/config/qtrobot-interface.yaml

    b. Change the line that says :code:`disable_interface: false` to :code:`disable_interface: true`

    c. Save and exit :code:`nano` by hitting Ctrl+x, then typing 'y', and then hitting Enter twice to confirm things.

.. note::

    You can reboot to see these changes take effect, or continue on and we'll reboot eventually.

Setting up our code
^^^^^^^^^^^^^^^^^^^

0. Secure-Shell (SSH) into QT's head computer::

    ssh qtrobot@192.168.100.1

1. Install our project's dependencies:

    .. parsed-literal::

        git clone -b |version| https://github.com/robotpt/abm-setup ~/abm-setup
        bash ~/abm-setup/scripts/pi_setup.bash

2. Increase the swap size, so we're able to build without running out of virtual memory:

    a. Turn off your swap memory::

        sudo /sbin/dphys-swapfile swapoff

    b. Open your swap configuration file::

        sudo nano /etc/dphys-swapfile

    c. Set `CONF_SWAPFACTOR` to 2 by changing the line that says :code:`#CONF_SWAPFACTOR=2` to :code:`CONF_SWAPFACTOR=2`, that is by deleting the :code:`#` character to uncomment the line. 

    d. Save and exit :code:`nano` by hitting Ctrl+x, then typing 'y', and then hitting Enter twice to confirm things.

    e. Turn the swap file back on::

        sudo /sbin/dphys-swapfile swapon

3. Clone our repositories and build them:

    a. Go to the source code directory in the catkin workspace::

        cd ~/catkin_ws/src

    b. Clone our repositories:

        .. parsed-literal::

            git clone -b |version| https://github.com/robotpt/cordial
            git clone -b |version| https://github.com/robotpt/qt-robot

    c. Build our workspace::

        cd ~/catkin_ws
        catkin_make

    .. note::

        It takes around five minutes for this command to finish.  You can setup QT's body computer at the same time as it runs, if you like.

4. Setup our code to run when QT's head computer turns on.

    a. Copy the autostart script into the correct directory::

        roscp qt_robot_pi start_usc.sh /home/qtrobot/robot/autostart/

    b. Enable the autostart script:

        i. Open a webbrowser on QT (e.g., Firefox) and go to `http://192.168.100.1:8080/ <http://192.168.100.1:8080/>`_.

        .. figure:: images/qt_menu.png
            :align: center

            QT's configuration menu.

        ii. Click 'Autostart'.  You'll be prompted for a username and password. Enter :code:`qtrobot` for both.

        iii. Click the 'Active' checkbox next to :code:`start_usc.sh`.

        .. figure:: images/autostart_checked.png
            :align: center

            QT's autostart menu with our script, :code:`start_usc.sh`, checked.

        iv. Click 'Save' and then 'Return' twice.

.. note::

    You can reboot to see these changes take effect, or continue on and we'll reboot eventually.

    If you'd like, you can confirm that things are running after a reboot by opening a terminal and running the following command.  You should see both :code:`/sound_listener` and :code:`/start_face_server`::

       rosnode list | grep "/\(sound_listener\|start_face_server\)"

    .. figure:: images/head_nodes_running.png
        :align: center

        What you should see if the head nodes are running correctly.

Body
----

Getting your Amazon Web Service credentials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For QT to speak, we use Amazon Polly, which requires an Amazon Web Services account. At our current usage, using `Amazon Polly is free up to a certain level <https://aws.amazon.com/polly/pricing/>`_), but you will need a credit card to create an account.

1. `Create an Amazon Web Services account <https://portal.aws.amazon.com/billing/signup#/start>`_.
2. Once you sign in, in the top right of the page, click your account name (mine says "Audrow"), then in the drop-down menu click "My Security Credentials," then click "Create New Access Key."
3. Record your access key and keep it somewhere safe.  You can do this by downloading this or just viewing it and copy-pasting it to somewhere for later reference.

.. note::

    It is best practice to create separate accounts with less access than your root account and use those access keys, see `Amazon's security best practices <https://aws.amazon.com/blogs/security/getting-started-follow-security-best-practices-as-you-configure-your-aws-resources/>`_.

Setting up an Amazon Web Service bucket
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For storing the recorded audio and video we'll use an Amazon Web services S3 bucket.

1. Login to `Amazon Web Services`_.

2. In "Find Services" type "S3" and click it when it appears.

3. Create your bucket:

    a. Hit the "Create bucket" button.

    b. Name your bucket and select US-West for the region.  Note that the name has to be globally unique, so you may have to add some random characters to it.

    c. Continue through the setup process leaving things as they are set by default (no public access, etc.) and finally click "Create bucket"

4. Write down the bucket name you have created.

Getting your Fitbit credentials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will need to make a Fitbit "app" for each Fitbit device.  We are interested in the Client ID, Client Secret, and a generated code that saves us from having to login on a web browser.

1. `Create a Fitbit account <https://dev.fitbit.com/login>`_ for each Fitbit device.

2. Login to your Fitbit account.

3. Go to `register an app <https://dev.fitbit.com/apps/new>`_

4. Fill in the application. You can put whatever you think makes sense for most of them (URL, policy, etc.).  (Make sure you include the `http` part int he urls.)  The following are the parts that matter to get access to the Intraday data.

    * "OAuth 2.0 Application Type" should be "Personal"

    * "Callback URL" should be `http://localhost`

    * "Default Access Type" should be "Read-Only"

    .. warning::

        If you get an error when trying to setup QT's body later, come back here and make sure things are correct.

    .. figure:: images/fitbit_application.png
        :align: center

        An example of what should be in the Fitbit app application

5. On the registered app's page, record your Client ID and Client Secret, and then click "OAuth 2.0 tutorial page," near the bottom.

    .. figure:: images/registered_app.png
        :align: center

        The registered app page.

6. On the Oauth2.0 tutorial page, set "Flow type" to "Authorization Code Flow."

    .. figure:: images/oauth2_tutorial.png
        :align: center

        Oauth2.0 tutorial page with "Flow type" set to "Authorization Code Flow."

    .. note::

        The "Expires In(ms)" text field is only used for "Implicit Grant Flow." "Authorization Code Flow," what we are using, expires in a fixed time (8 hours), but we are able to renew our authorization.

7. Click the URL above "1A Get Code." You'll be brought to an error page, but that's okay.  We need the code from the URL. Record that code.

    .. figure:: images/fitbit_code.png
        :align: center

        The page that you arrive at when clicking the URL above "1A Get Code."  The code we are interested in in the URL is highlighted.

    .. warning::

        If the URL is longer than in the picture, go back to the OAuth2.0 tutorial page and make sure that you have the "Flow type" set to "Authorization Code Flow," not "Implicit Grant Flow."

    .. note::

        The code obtained in this step only works once.  After you use it to initialize a Fitbit client, it cannot be used again.  We use it to obtain an access and refresh token for talking to Fitbit's web API.  If you need to reset Fitbit credentials for any reason, you will have to go to the OAuth2.0 tutorial page and get a new code.

.. note::

    From this section, you should have the following information:

        * Client ID
        * Client Secret
        * A generated code


Setting up our interaction
^^^^^^^^^^^^^^^^^^^^^^^^^^

0. Change your system timezone to be in your current timezone.  To do this, you can click the time in the upper-right of the desktop on QT and then click 'Time & Date settings...'

1. Open a terminal and clone this repository onto QT's body computer:

    .. parsed-literal::

        git clone -b |version| https://github.com/robotpt/abm-setup ~/abm-setup

2. Run a script to allow for updates::

    sudo bash ~/abm-setup/scripts/nuc_setup.bash

.. warning::

    If this step fails, try the following commands before rerunning::

        sudo apt install --reinstall python3-six
        sudo apt install --reinstall python3-chardet

.. note::

    This step takes five minutes or so.

3. Setup Docker:

    a. Install Docker::

        curl -fsSL https://get.docker.com -o get-docker.sh
        sh get-docker.sh

    b. Set Docker to run without :code:`sudo`::

        sudo groupadd docker
        sudo gpasswd -a $USER docker
        newgrp docker

    c. Test that Docker is installed correctly and works without :code:`sudo`::

        docker run hello-world

    .. figure:: images/hello_from_docker.png
        :align: center

        What is printed from running the :code:`hello-world` docker container.


4. Setup Docker-compose:

    a. Install Docker-compose::

        sudo curl -L "https://github.com/docker/compose/releases/download/1.25.3/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        sudo chmod +x /usr/local/bin/docker-compose

    b. Check that docker compose is installed correctly::

        docker-compose version


5. Setup the docker container:

    .. note::

        The first time that you run the Docker script, it will take around 15 minutes to setup the container.  After that, it will be fast.  Feel free to take a break or go get coffee :-)

    a. Open the :code:`Dockerfile` with :code:`nano ~/abm-setup/docker/Dockerfile` and replace the name of the Amazon Web Services bucket. The line to change is at the bottom of the file and should be changed to :code:`ENV AWS_BUCKET_NAME <your aws bucket's name>` with whatever your bucket is named, for example, :code:`ENV AWS_BUCKET_NAME qt-robot-1`.

    b. Run the :code:`docker.sh` script with the :code:`setup` option::

        bash ~/abm-setup/docker/docker.sh setup

    .. note::

        I did have an error occur during this command one of the times I was setting it up.  It might have been a network issue.  I ran it again and it succeeded.  If you have trouble here let me know.

    d. Enter your Fitbit and Amazon Web Services credentials as prompted.  The following is the order they are asked in and what they look like / should be:

        .. list-table::
           :header-rows: 1
           :align: center

           * - Prompt
             - Example / value
           * - Fitbit Client ID
             - :code:`22XXXX`
           * - Fitbit Client Secret
             - :code:`5912f5907faa693e3e6630XXXXXXXXXX`
           * - Fitbit *Ultra Secret* Code
             - :code:`6e843fa2b908b1f608b973b845b793XXXXXXXXXX`
           * - AWS Access Key ID
             - :code:`AKIAY2SYU4XXXXXXXXXX`
           * - AWS Secret Access Key
             - :code:`jwY9mv9U7DBfZe2/p5XXXXXXXXXXXXXXXXXXXXXX`
           * - AWS Default Region Name
             - :code:`us-west-1`
           * - AWS Default Output Format
             - :code:`json`

        .. warning::
            If you receive an error after entering the Fitbit information, check that you have a device setup with the Fitbit account.

    e. Ignore the network information displayed and hit Ctrl+C to close the container.


6. Run the interaction:

    a. Make sure that you're in the :code:`docker` directory in the :code:`abm-setup` folder::

        cd ~/abm-setup/docker

    b. Run the :code:`docker.sh` script with the :code:`run` option::

        bash docker.sh run

    .. figure:: images/docker_run.png
        :align: center

        An example of the final message after the interaction run script.


7. Make the interaction run on startup:

    a. List your Docker containers::

        docker container ls

    .. figure:: images/docker_container_list.png
        :align: center

        An example of running containers.

    b. Copy the "CONTAINER ID".

    c. Update the container's restart policy::

          docker container update --restart=unless-stopped <YOUR COPIED CONTAINER ID>

.. note::

    At this point, you should reboot QT.  You can do this by either pushing the button on the back of QT or typing :code:`sudo reboot` into the head computer's terminal.

    To test that things are setup correctly, you can take the URL for the GUI that you wrote down and type it into the web-browser on any device that's on the same network.  QT should begin asking you about your name, if it is your first interaction.

Tablet
------

For either tablet supplied by LuxAI with QT, or any Android tablet for that matter, we're going to set up the tablet to run as a Kiosk using the app `Fully Kiosk Browser <https://www.ozerov.de/fully-kiosk-browser/>`_.

1. Sign on to the Google Play Store.

2. Search for and download `Fully Kiosk Browser`.

3. Go to settings and connect to QT's network, for example, :code:`QT145`.  The password should be :code:`11111111` (eight ones).

4. Start `Fully Kiosk browser` and set the start URL 192.168.100.2:8082.

5. Adjust settings in `Fully Kiosk browser`:

    i. In 'Settings > Web Zoom and Scaling', disable 'Enable Zoom'

    ii. In 'Settings > Web Auto Reload', set 'Auto Reload after Page Error' to '2'.

With this app, you can make it so that it's challenging to get out of the app or do other things on the tablet.  You can go into 'Settings > Kiosk Mode (PLUS)' to play with these settings.  A plus license is 6.90 EUR per device (about 7.50 USD).

Reseting QT
===========

Full reset
----------

If you would like to delete the data stored on QT, as well as reset the Fitbit and AWS credentials, enter the following commands from QT's body computer::

    cd ~/abm-setup/docker
    docker-compose down -v

Reset AWS credentials
---------------------

Just run the setup, again::

    bash ~/abm-setup/docker/docker.sh setup
    
Reset Fitbit Credentials and/or interaction history
---------------------------------------------------

I will seek to make this easier for the full deployment, but for now, do the following:

1. Open a terminal to the Docker environment::

    bash ~/abm-setup/docker/docker.sh debug

2. Remove what you'd like:

    a. Remove the Fitbit credentials document from the terminal that pops up::

        rm /root/state/fitbit_credentials.yaml
    
    b. Remove the interaction history to start again from QT introducing itself and setting up the interaction::
    
        rm /root/state/state_db.pkl
    
3. Exit the Docker terminal (you can just close it).

To setup your Fitbit credentials, in your original terminal, run the setup script again::

    bash ~/abm-setup/docker/docker.sh setup
    
.. note::

    The Amazon Web Services credentials will show that they have values with the values in brackets (e.g., :code:`[XX..XXJUXB]`).  You can just hit *Enter* to leave these values unchanged.
