Updating QT
===========

Most updates to QT will only involve the body computer (Audrow should tell you otherwise).
In this case, the update process should be fairly straightforward.

Turn on QT, open a terminal, go to the :code:`abm-setup` directory, and pull the updates::

    cd ~/abm-setup
    git pull

Then start the interaction as specified in the setup.
The container rebuild and the updates will be applied.

If you are updating the head computer, Audrow will provide specific instructions.
Most likely, you will have to just go through the updated setup for the head computer.
Audrow should let you know.

Preparing QT for new participants
=================================

To prepare QT for new participants, we have to get the data off and reset the data stored (not the AWS or Fitbit credentials, those can stay as they are).

Getting the data off QT
-----------------------

To get the data off QT, turn on QT, open a terminal, and open a terminal to QT's Docker container::

    cd ~/abm-setup/docker
    docker-compose down
    bash docker.sh debug

From the terminal that opens with the command :code:`bash docker.sh debug`, copy the data to a folder that is shared between the Docker container and QT's computer::

    zip -r /root/shared/log_$(date "+%Y-%m-%d_%H:%M:%S") /root/mongodb_log/ /root/state/

In the other terminal (not the one that opened with :code:`bash docker.sh debug`), go to the shared directory and change the file owner.
The final command opens a file explorer for convenience::

    cd ~/abm-setup/docker/shared
    sudo chown $USER *
    xdg-open .

.. note::

    As a sanity check you can unzip the file now and check its contents.

From here you should save the zip file elsewhere, on a harddrive, on the cloud, etc.

Now we can delete the files that save the state of the interaction and user interaction log in our terminal that we started with :code:`bash docker.sh debug`.

    rm -r /root/state/* /root/mongodb_log/*

Now start the interaction as per the setup instructions and QT is ready to begin with a new participant.

Note that, QT will look at the last seven days of Fitbit activity with the associated Fitbit device in setting the first week's steps goal, so if you want to go straight to a new participant, maybe you should use a different Fitbit account.