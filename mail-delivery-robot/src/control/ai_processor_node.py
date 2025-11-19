import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ollama import chat
except ImportError:
    chat = None

"""
ros2 run mail-delivery-robot captain
ros2 run mail-delivery-robot ai_processor_node
Basically subscribes to existing topics that robot publishes, creates new 
topics for AI responses. ex."/ai/battery_advice", "/ai/captain_advice"

Create new topics vs. Publish to existing topics

Create new topics:
Pros:
- no feedback loop (if AI node subscribes to a topic and republishes to same topic
, has risk of reading its own message updating itself)
- can compare AI output to non AI output
- easy to swap AI nodes without breaking existing nodes

Cons:
- more topics
- nodes that want to use AI needs to subscribe to the new topic

Publish to existing topics:
Pros:
- Other nodes already subscribe to those topics so they will automatically get
the AI process data, makes things easier
- less clutter in topic list

Cons:
- Risk of feedback loops
- Harder to debug, cant tell if its AI sent message or not, unlesss extra logging
is added
- cant easily compare "raw data" vs "AI data"
"""

class OllamaChat:
    def __init__(self, model="qwen3", stream=False, think=False):
        self.model = model
        self.stream = stream
        self.think = think

    def ask(self, prompt: str) -> str:
        if chat is None:
            return f"[ERROR] 'ollama' module not installed. Prompt: {prompt}"
        try:
            response = chat(
                model=self.model,
                messages=[{"role": "user", "text": prompt}],
                think=self.think,
                stream=self.stream,
            )
            return response.messages.content.strip()
        except Exception as e:
            return f"[ERROR] Failed to get AI response: {e}"


class ai_processor_node(Node):
    """
    general purpose AI Processor Node, subscribes to multiple topics
    basically send data to LLM and publish responses
    If you want you can have each topic have its own prompt template and output topic
    """
    def __init__(self):
        super().__init__("ai_processor_node")

        self.model = "qwen3"
        self.think = False
        self.stream = False
        # can make your own config
        # input = topic to sub
        # output = topic to pub to AI
        # prompt = your prompt for the LLM
        self.topics = [
            {
                "input": "/battery_status_text",
                "output": "/ai/battery_advice",
                "prompt": "Battery report: {data}. Summarize what the robot should do next."
            },
            {
                "input": "/captain_status",
                # if you want to inject AI advice directly into existing topics
                # for example /actions for captain change output to "output": "/actions"
                # if want to create new topic "output": "/ai/captain_advice"
                "output": "/ai/captain_advice",
                "prompt": "Here are the current robot actions: {data}. Suggest the best next action."
            }
        ]

        # Initialize AI interface
        self.ollama = OllamaChat(self.model, think=self.think, stream=self.stream)
        # Dictionary to hold publishers (avoid ROS2 Node.publishers conflict)
        self.ai_publishers = {}

        for topic in self.topics:
            input_topic = topic["input"]
            output_topic = topic["output"]
            prompt_template = topic["prompt"]

            pub = self.create_publisher(String, output_topic, 10)
            self.ai_publishers[input_topic] = pub

            self.create_subscription(
                String,
                input_topic,
                lambda msg, it=input_topic, pt=prompt_template: self.process_message(it, pt, msg),
                10
            )
            self.get_logger().info(f"Listening on {input_topic}, publishing to {output_topic}")

    def process_message(self, topic_name: str, prompt_template: str, msg: String):
        text = msg.data.strip()
        prompt = prompt_template.format(data=text)

        self.get_logger().info(f"[{topic_name}] Prompt: {prompt}")
        response = self.ollama.ask(prompt)

        out_msg = String()
        out_msg.data = response
        self.ai_publishers[topic_name].publish(out_msg)
        self.get_logger().info(f"[{topic_name}] AI Response: {response}")


def main(args=None):
    rclpy.init(args=args)
    node = ai_processor_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
