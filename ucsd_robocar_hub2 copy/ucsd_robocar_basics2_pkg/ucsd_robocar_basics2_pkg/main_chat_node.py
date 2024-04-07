import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from openai import OpenAI

# Directly setting the API key
api_key = "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"
client = OpenAI(api_key=api_key)

NODE_NAME = 'main_chat_node'
conversation_history = []  # Maintain conversation history 

def chat_with_gpt(prompt):
    global conversation_history 

    # Incorporate conversation history into the prompt
    enhanced_prompt = "".join(conversation_history) + prompt  
    conversation_history.append(f"User: {prompt}\n")  # Add user input to history

    try:
        completion = client.chat.completions.create(
            model="gpt-3.5-turbo", 
            messages=[
                {"role": "user", "content": enhanced_prompt},
            ]
        )
        response = completion.choices[0].message.content.strip()
        conversation_history.append(f"Chatbot: {response}\n")  # Add chatbot response to history
        return response
    except Exception as e:
        print(f"An error occurred: {e}")
        return "Sorry, I couldn't process your request."


class ChatBotNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            String,
            'chat_input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        user_input = msg.data  # The message data is the user input
        response = chat_with_gpt(user_input)
        self.get_logger().info('Chatbot: "%s"' % response)

def main(args=None):
    rclpy.init(args=args)
    chat_bot_node = ChatBotNode()
    try:
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(chat_bot_node)
        try:
            executor.spin()
        finally:
            chat_bot_node.get_logger().info(f'Shutting down {NODE_NAME}...')
            # If you had any additional cleanup specific to your node, it would go here.
            chat_bot_node.get_logger().info(f'{NODE_NAME} shut down successfully.')
            executor.shutdown()
            chat_bot_node.destroy_node()
    except KeyboardInterrupt:
        pass  # This block allows the program to exit gracefully on a keyboard interrupt without printing an error message.
    finally:
        rclpy.shutdown()



if __name__ == "__main__":
    main()
