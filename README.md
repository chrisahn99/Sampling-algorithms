# Sampling-algorithms

This is the code repo for the final project for the course AI614 - Robot Task and Motion Planning by Professor Beomjoon KIM at KAIST.

- **sampling_base.py** is where all the main classes for creating the map and generating the sampling pathes are found. 
- **alternative1.py** and **alternative2.py** are respectfully the codes for 1.1 and 1.2.
- **RRT.py** is for 1.3.

Here is the playlist link for execution videos:
https://youtube.com/playlist?list=PLIPEX2JEd82G2C6wATwLy-CNTBQYpq9rE

You can play around with the following parameters:
- *dmax* argument in the **step** function in **sampling_base.py**
- obstacle number
- obstacle dimension
- map dimension
- start and goal coordinates

Also, RRT_bias.py is a bonus algorithm combining RRT and domain knowledge exploitation inspired by 1.2.
