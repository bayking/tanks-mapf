# Assignment 2 

Assignment 2 – Theme "Reasoning and Planning", (ch.7-17, AIMA)

## Introduction

This project aims to solve the MAPF problem with use of the algorithms Conflict-based Search and Prioritized Planning. It is an altered version of **Kjell Erik Näckros** tanks_190324 code base. 

The Conflict-Based Search algorithm is heaviliy based on the paper [Conflict-Based Search For Optimal Multi-Agent Path Finding](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239) by G. Sharon, R. Stern, A. Felner and N. Sturtevant.

## MAPF algorithms

### Conflict-Based Search 

CBS is a two-level algorithm. At the high level, a search is performed on a tree based on conflicts between agents. At the low level, a search is performed only for a single agent at a time.

### Prioritized Planning

Given an agents priority a path is generated. The following agents paths must avoid the positions and time points that have been reserved by previous agents.

## Prerequisites

This project is built in Processing for Java. It will not work with older versions of Processing.

	* Processing 4.0 alpha 1

## Install Processing

Before you start with this project, Processing 4 has to be installed.

Go to the [Processing download page](https://processing.org/download/) and download the correct Pre-Release version for your operating system.

## Structure
* The world (Playing field) is 800  x 800 pix.
* A tank occupies a surface corresponding to a cirle with a 50 pix diameter. 
* A tanks maxspeed is 3pix per (1/60) s. ( frameRate(60) )
* Starting positions for red tanks: [(50,50), (50,150), (50,250)], blue tanks: [(width-50, height-250), (width-50, height-150), (width-50, height-50)]
* Homebase position, red: (x:0, y:0, w:150, h:350), blue: (x:width-151, y:height-351, w:150, h:350)
* Obstacles, 3 trees.(center points): [(230, 600), (280, 230), (530, 520)], diameter: 165
* A tank may move forward 50px, backwards 50px, right 50px, or left 50px.
* A tank may not go outside of the playing field. 
* An obstacle (tree or another tank) is an object that tanks may not pass through, see through or impact with force.
* All tanks knows about the positions of the obstacles.
* When a tank has reached its goal, it will stay there.
	
### Goal

The blue teams(Team2) agents are given goal states that they have to reach without colliding with themselves or any other obstacles.

## Usage

Start Processing 4 and open the project.

Team2 holds a Search object in which the two algorihtms are defined in methods. To shift between them, change line 33 in Team2 to:

### Conflict-based Search
```
solutions = search.csb();
```

### Prioritized Planning
```
solutions = search.pp();
```

The goals and starting positions of the agents may be altered in the constructor of Team2.

## Built With

* [Processing 4.0.a1](https://github.com/processing/processing4) - The Java framework used

## Known bugs

When a tank has reached its goal position, that position will be considered empty in later time points which may result in collisions.

## Authors

### Grupp 7
* **Mustafa Bay**
* **Maximilian Törnqvist**
* **Fredrik Hammar**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Kjell Erik Näckros
* Guni Sharon
* Roni Stern
* Ariel Felner
* Nathan Sturtevant
