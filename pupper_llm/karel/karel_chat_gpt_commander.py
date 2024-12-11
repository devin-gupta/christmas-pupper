import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
from openai import OpenAI
import karel  # Importing your KarelPupper API
import openai
from pydantic import BaseModel

client = OpenAI(api_key='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')  # Set your OpenAI API key here

class Backward(BaseModel):
    steps: int

class Forward(BaseModel):
    steps: int

class TurnLeft(BaseModel):
    degrees: int

class TurnRight(BaseModel):
    degrees: int

class Bark(BaseModel):
    pass

class Stop(BaseModel):
    pass

tools = [
    openai.pydantic_function_tool(Backward),
    openai.pydantic_function_tool(Forward),
    openai.pydantic_function_tool(TurnLeft),
    openai.pydantic_function_tool(TurnRight),
    openai.pydantic_function_tool(Bark),
    openai.pydantic_function_tool(Stop),
]

class GPT4ConversationNode(Node):
    def __init__(self):
        super().__init__('gpt4_conversation_node')

        # Create a subscriber to listen to user queries
        self.subscription = self.create_subscription(
            String,
            'user_query_topic',  # Replace with your topic name for queries
            self.query_callback,
            10
        )

        # Create a publisher to send back responses
        self.publisher_ = self.create_publisher(
            String,
            'gpt4_response_topic',  # Replace with your topic name for responses
            10
        )

        # vis node
        self.vis_publisher_ = self.create_publisher(
            String,
            'visualization_topic',  # Replace with your topic name for responses
            10
        )

        self.get_logger().info('GPT-4 conversation node started and waiting for queries...')

        # Initialize the text-to-speech engine
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # Set the speed of speech (optional)

        # Initialize KarelPupper robot control
        self.pupper = karel.KarelPupper()

    # TODO: Implement the query_callback method
    # msg is a String message object that contains the user query. You can extract the query using msg.data
    def query_callback(self, msg):
        # pass
        # Paste in your implementation from simple_gpt_chat.py
        # Extract the user query from the message
        query = msg.data

        # Call GPT-4 API to get the response
        response = self.get_gpt4_response(query)

        # Create a new String message for the response
        response_msg = String()
        response_msg.data = response

        # Publish the response
        self.publisher_.publish(response_msg)

        # DEBUG LOGGERS
        self.get_logger().info(f"Received user query: {query}") 
        self.get_logger().info(f"Published GPT-4 response: {response}")

        
        # Play the response through the speaker with the play_response method
        self.play_response(response)
        # Parse and execute robot commands if present with the execute_robot_command method
        self.execute_robot_command(response)

    def get_gpt4_response(self, query):
        self.get_logger().info(f"query: {query}")
        try:
            # Making the API call to GPT-4 using OpenAI's Python client
            prompt = '''
            You are a walking assistant that translates human language into an array of KarelPupper commands. When given instructions,
            use the TurnRight or TurnLeft tools for rotation commands, Forward, Backward or Stop tools for movement controls and an optional Bark tool if appropiate.
            This could include multiple of each command. For example, with input 'Pupper walk 3 steps forward' you can do: 
            Forward, Forward, Forward, Bark.
            '''
            response = client.chat.completions.create(model="gpt-4",  # Model identifier, assuming GPT-4 is used
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": query}
            ],
            tools=tools,
            max_tokens=150)  # Adjust token limit based on your requirement

            tool_calls = response.choices[0].message.tool_calls

            functions_called = []

            if tool_calls:
                for tool_call in tool_calls:
                    print(tool_call.function.name)
                    functions_called.append(tool_call.function.name)
                    self.execute_robot_command(tool_call.function.name)

            # Extract the assistant's reply from the response
            # gpt4_response = response.choices[0].message.content
            if len(functions_called) > 0:
                result = "Ok, I will " + " ".join(functions_called)
            else:
                result = 'play a game with me'
            self.get_logger().info(f"result {result}")
            return result

        except Exception as e:
            self.get_logger().error(f"Error calling GPT-4 API: {str(e)}")
            return "Sorry, I couldn't process your request due to an error."

    def play_response(self, response):
        try:
            # Use the TTS engine to say the response out loud
            self.tts_engine.say(response)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"Error playing response through speaker: {str(e)}")

    def execute_robot_command(self, response):
        # Convert the response to lowercase to handle case-insensitivity
        response = response.lower()
        self.get_logger().info(f"Response: {response}")
        # TODO: Implement the robot command execution logic, in a large if-else statement. Your conditionals should be set based on the expected commands from GPT-4, and the corresponding methods should be called on the KarelPupper object.
        match response:
            case "forward":
                print('pupper forward')
                self.pupper.move()
            case "turnleft":
                print('pupper turn left')
                vis_resp = String()
                vis_resp.data = 'look_left'
                self.vis_publisher_.publish(vis_resp)
                self.pupper.turn_left()
            case "turnright":
                vis_resp = String()
                vis_resp.data = 'look_right'
                self.vis_publisher_.publish(vis_resp)
                self.pupper.turn_right()
            case "bark":
                print('pupper barking!')
                vis_resp = String()
                vis_resp.data = 'blink'
                self.vis_publisher_.publish(vis_resp)
                self.vis_publisher_.publish(vis_resp)
                self.pupper.bark()
            case "backward":
                print('pupper backward')
                self.pupper.move()

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    gpt4_conversation_node = GPT4ConversationNode()
    rclpy.spin(gpt4_conversation_node)

    # Clean up and shutdown
    gpt4_conversation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
