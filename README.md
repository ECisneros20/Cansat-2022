# Cansat-2022

## Description

Simplified version of the satellite container and payload code part of the Cansat USA 2022 competition.

This includes the initial configuration of all sensors and their respective codes to process and print their information in a comma-separated data frame. In addition, each of these printed lines is saved in a csv file.

## Installation

To clone the latest stable version:

	git clone https://github.com/ECisneros20/Cansat-2022.git

To clone the latest version:

	git clone -b testing https://github.com/ECisneros20/Cansat-2022.git

## Usage

First, you must have all the prerequisite libraries:

	pip install -r requirements.txt

Then, run the following script to the container computer:

	python container.py

Finally, run the following script to the payload computer:

	python payload.py

In case, you want to check programming functionality without sensors usage:

	python container-no-sensors.py

	python payload-no-sensors.py

## Next steps

1. Include the capabilities of communication with radio modules between container-payload and container-groun station.

## License

MIT License
