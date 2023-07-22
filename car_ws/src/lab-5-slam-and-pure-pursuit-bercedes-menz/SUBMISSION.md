# Lab 5: SLAM and Pure Pursuit

## Map files

[levine_2nd pgm + yaml](https://www.dropbox.com/sh/aws4809bygg8vdi/AAAxczTIAa4i5CUbzz55ZRcya?dl=0)

## YouTube video link

[Actual Car](https://youtu.be/ZS8i81iUaQU)

[Sim](https://youtu.be/VgiZiXVL_kA)

## Lab 5 dependencies:

We use a custom python package to run the controllers of the car. 
You can find it in this dropbox link: https://www.dropbox.com/sh/dihvcnr9v9trq8r/AAD_VPWwayCCpjyDf8xAK-R5a?dl=1 

To install:
```bash
cd ~
curl -L -o libf1tenth.zip https://www.dropbox.com/sh/dihvcnr9v9trq8r/AAD_VPWwayCCpjyDf8xAK-R5a?dl=1 --http1.1
mkdir libf1tenth
unzip libf1tenth.zip -d libf1tenth
cd libf1tenth
pip install -e .
```
