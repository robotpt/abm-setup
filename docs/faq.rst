Frequently Asked Questions
==========================

On video recording
------------------

Is the interaction recording?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default the interaction is being recorded.  If the interaction is proceeding, it means that the Amazon Credentials are correct and that the AWS bucket is valid.

How do we access the recordings / make sure that the face is in view?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All recordings are stored on Amazon Web Services.
Get the credentials from Audrow.
You can then go to S3 storage on AWS and view the contents of each bucket to see if the face is in frame.

On implementation
-----------------

How are steps goals calculated?

.. math::

    g_\text{this week} = \max\left(g_\text{week min}, ~\min\left(g_\text{week max}, ~\hat{g}_\text{this week}\right)\right)

    \hat{g}_\text{this week} = s_\text{past week} + \frac{g_\text{final week} - s_\text{past week}}{ \text{# weeks remaining}}

    g_\text{week min} = \max(g_\text{week goal lower bound},~\alpha_\text{min}~\cdot~s_\text{past week})

    g_\text{week max} = \alpha_\text{max}~\cdot~s_\text{past week}

Where :math:`g` refers to active steps goal, :math:`s` refers to the number of steps completed by the participant, and :math:`\alpha_\text{min}` and :math:`\alpha_\text{max}` are constants such as 1.1 and 2.0.

Note that :math:`g_\text{this week} = \alpha_\text{min}~\cdot~s_\text{past week}` if :math:`g_\text{this week} \geq g_\text{final week}`.

The daily steps goal is calculated as follows:

.. math::

    g_\text{today} = \max\left(g_\text{today min}, 
 ~\min\left(g_\text{today max},~ \hat{g}_\text{today}\right)\right)

    \hat{g}_\text{today} = \frac{g_\text{this week} - s_\text{this week}}{\text{# days remaining}}

    g_\text{today min} = \frac{g_\text{this week}}{\text{# days per week}}

    g_\text{today max} = \gamma \cdot g_\text{today min}

Where :math:`\gamma` is a constant that relates :math:`g_\text{today min}` to :math:`g_\text{today max}`, for example, the value 2.5.
:math:`\gamma` prevents the days recommendation from being enormous if they haven't done well during the week.
