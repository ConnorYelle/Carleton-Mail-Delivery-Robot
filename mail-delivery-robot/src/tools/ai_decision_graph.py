from langgraph.graph import StateGraph, END
import ollama

class NavState(dict):
    current_beacon: str
    destination: str
    direction: str | None

def build_nav_graph(node):

    #llm = Ollama(model="qwen2.5:0.5b", temperature=0.1)

    graph = StateGraph(NavState)

    def decide_direction(state: NavState):
        prompt = f"""
        The robot is currently at beacon {state["current_beacon"]} and needs to go to {state["destination"]}. 
        What direction should it take next from the following list of options based on the current state and map of the tunnels?:
        - NAV_LEFT
        - NAV_RIGHT
        - NAV_PASS
        - NAV_U-TURN
        - NAV_DOCK
        Provide only one of these directions as the output.
        """
        node.get_logger().info(f"Prompt to LLM: {prompt}")

        response = ollama.generate(
            model="gemma2:2b-instruct-q4_0",
            prompt=prompt
        )

        node.get_logger().info(f"Raw LLM Response: {response}")

        # Extract only the first token-like output, in case model returns explanation
        result = response["response"].strip().split()[0]

        node.get_logger().info(f"LLM Response: {result}")

        state["direction"] = result
        return state

    
    graph.add_node("Decide Direction", decide_direction)
    graph.set_entry_point("Decide Direction")
    graph.add_edge("Decide Direction", END)

    return graph.compile()

if __name__ == "__main__":
    pass