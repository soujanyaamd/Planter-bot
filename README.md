# Planter-bot #

A Project that secured third prize in E-Yantra Robotics Competition, 2018

The team members:
1. Soujanya Avadhani M D
2. Akshatha S
3. Krupa Sindhu S
4. Pragna G Shastry

#############################################

The need for automation in agriculture and allied activities is increasing manifold. This is due to factors such as: lack of labour caused by migration to cities, health hazards posed by working on the farm using one’s bare hands in different climates and one's own capacity to do physical work. In spite of these deterrent factors, nearly 58% of the rural population is engaged in some kind of agricultural activity for their livelihood. Given the fact that agriculture is one of the major contributors to GDP in India, the situation is compelling to be taken more seriously and think of solutions to automate various farming activities.

In accordance to this, E-Yantra Robotics Competition, 2017-18 had seven agriculture based themes, aimed at automating the various processes in a farm. These themes were abstractions of different agricultural processes such as sowing the seeds, harvesting the crops and transporting the produce to the market.

One such theme was ‘Planter Bot’ to address the most preliminary and basic farming task of seedling and sapling plantation. The idea was to build a robot which will be able to traverse the field with good accuracy in path detection with the aid of vision. The robot has to traverse a path having different Zones growing distinct types of plants. Each Zone has an associated requirement for seedlings of a particular kind to be planted. The robot has to plant the seedlings as per the requirement in each Zone which is simulated using image overlay. The challenges in this theme included: designing and building a robot with the basic components given, Python programming and Image Processing.
Here, the "path" corresponds to a white arena with black lines which the robot should follow to reach destination. The "seedlings" are identified by the number, color and shape of the color markers placed on either side of the line. "Planting" of the seedlings is done by overlaying the image of the corresponding seedlings on the plantation background image.

Our team took part under the same theme, ‘Planter Bot’. In order to achieve all the above mentioned objectives, we have designed an Open CV - Python based bot. The Pi Camera placed on the bot captures the frames at the rate of 60 fps and resolution of 5 mega Pixel and sends them to raspberry-pi. The raspberry-pi placed on the bot effectively handles all the processing. To follow the path accurately, the bot uses centroid based algorithm. The bot runs at a speed of 75 rpm. Smooth turns are implemented at curved paths and differential turns are implemented at right angled paths. To detect the various plantation zones, area thresholding was used. Thus, prototype of a ‘planter bot’ is developed.

###############################################

The folders included in this repository are:

1. The "task" folder which has two sub-folders, "task_A" and "task_B". 
   task_A was to identify the different shapes in an image and recognise their color and centroid. The test images and final output        images are included. 
   task_B was to overlay the given images of flowers onto the corresponding shape of corresponding color such that it fits the shape        properly. The overlay images of the flowers are included.

2. The final_task folder contains the final code which was used to run the robot at the competition. The seedling images of the flowers      and the background image are included in the folder.

