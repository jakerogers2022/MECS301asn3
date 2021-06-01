# CS_ME_301 Machine Learning Project
**=====================================**

**Abstract**

In this project, we decided to use the KNN machine learning algorithm to predict the appropriate robot behavior in
various obstacle situations. We created two datasets, and tested the model
using the datasets and previously developed reactive controllers. We found that
the dataset with the least variability performed the best, and that the 
reactive controllers performed the worst.

**Data Collection Methods**

We used a single CoppeliaSim scene for our
data collection. 

We created two datasets: *Center Only* and *Mixed Positions*.
These datasets can be found in the **MLRobotics.py** file.

For Center Only, we placed the robot in the center of a cell,
in which a cell is four adjacent squares in CoppeliaSim. Then
we placed walls around the edges of a cell in various positions,
and recorded the sensor data as the features, and recorded the appropriate 
behavior (turn left, turn right, walk forward, or turn around) as the label.

The same collection was done for Mixed Positions, except that the robot 
be could be in any of the four corners of a cell, not just the center.

Each dataset is a dictionary with this key-value schema:

The keys are *X vectors*, X vectors are tuples of form:
                            (front sensor val, left sensor val, right sensor val)

The values are *Y vectors*, Y vectors are strings of one of the following:
     "F" - front, "L" - left, "R" - right, "T" - turn around

**Pipeline Framework and Execution**

The **MLRobotics.py** file has the **KNN class** with the two datasets.

It contains a data split function that given a training proportion and datset as input,
returns the dataset randomly partitioned into training and testing portions, according to the inputted
training proportion. For example, if the training proportion is 80%, it returns 80% of the dataset
as the training dataset, and the remaining 20% as testing (this testing portion is then split into its features and labels).

This class also runs the KNN prediction using the mode function as the aggregator for
K values that are greater than 1. The mode function returns the behavior that appears the most,
and in the case of a tie, returns the behavior for which its X vector is closest to the 
vector that is being predicted.

This class also contains an evaluation function that returns the raw accuracy of the 
predictions against the testing labels.

The **ReactiveControlTesting.py** file was used to test the pre-exisitng reactive controllers against the KNN model.
The **asn3.py** file contains the behaviors the control the robot.


