# Christmas-Themed Robotic Dog: Santa Says

Welcome to the **Christmas-Themed Robotic Dog** project! This project brings holiday cheer with a robotic dog that listens to voice commands starting with "Santa Says" and performs corresponding actions. Using advanced AI and robotics libraries, the dog can walk, turn, bark, and more, making it a fun and interactive way to celebrate the festive season.

## Project Highlights

- **Game Mechanic**: Plays "Santa Says" by executing commands only if they start with "Santa Says."
- **Voice Recognition and Command Parsing**: Leverages OpenAI's GPT-4 and function-calling APIs to translate human instructions into actionable robot commands.
- **Robotic Control**: Powered by the KarelPupper library for movement and interactions.
- **Christmas Spirit**: Includes festive actions like barking on command and following directions for enhanced holiday fun.

## Dependencies

To set up and run this project, you'll need the following:

1. **Python 3.8+**
2. **ROS 2 Foxy** (for robot control and messaging)
3. **Libraries**:
   - `rclpy` (ROS 2 Python client library)
   - `std_msgs` (ROS standard messages)
   - `pyttsx3` (Text-to-speech engine)
   - `openai` (for GPT-4 API integration)
   - `karel` (KarelPupper robot library)
   - `pydantic` (for data validation and modeling)

### Installation

Follow these steps to install dependencies:

1. Clone this repository:

   ```bash
   git clone https://github.com/devin-gupta/christmas-robotic-dog.git
   cd christmas-robotic-dog
   ```
2. Install Python dependencies via requirements.txt file:

   ```bash
   pip install -r /path/to/requirements.txt
   ```
3. Install ROS 2 Foxy (instructions [here](https://docs.ros.org/en/foxy/Installation.html)).
4. Install the KarelPupper library (refer to the [KarelPupper documentation](https://github.com/stanfordroboticsclub/karel-pupper-api) for setup).
5. Set your OpenAI API key as an environment variable:

   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

## How to Run

1. Start a ROS 2 workspace and source it:

   ```bash
   source /opt/ros/foxy/setup.bash
   ```
2. Run the main Python script:

   ```bash
   python3 main.py
   ```
3. Interact with your robotic dog by publishing voice commands to the `user_query_topic` topic in ROS 2. Use commands like:

   - "Santa Says walk forward."
   - "Santa Says turn left."
   - "Santa Says bark."

   The robot will only respond to commands that begin with "Santa Says" and ignore all others.

## Demo

![Demo Picture](path/to/demo_image.jpg)
[Link to Demo Video](https://www.youtube.com/watch?v=demo)

## Presentation Slides

[Project Presentation Slides](https://docs.google.com/presentation/d/1BMILGRh6PE5-_HEO-lYg67qFrJpFU2cOq54jUiqWgxU/edit?usp=sharing)

## Features in Development

- Enhancing command parsing with more festive phrases.
- Adding additional Christmas-themed actions.
- Improving the robot’s physical design for a more festive look.

## Acknowledgments

- [OpenAI GPT-4](https://openai.com)
- [KarelPupper](https://github.com/karelpupper/karelpupper)
- ROS 2 Community

Spread the joy of Christmas with your very own **Santa Says Robotic Dog**! Feel free to contribute and make this project even better.
