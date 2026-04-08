from langgraph.graph import StateGraph, END
import ollama
from .map import Map

class NavState(dict):
    current_beacon: str
    destination: str
    direction: str | None

def build_nav_graph(node):

    #llm = Ollama(model="qwen2.5:0.5b", temperature=0.1)

    graph = StateGraph(NavState)
    
    mapping = Map()

    map = """
    Map of tunnels:
    ,Loeb,Southam,TunnelJunction,Library,Azrieli/Tory,Steacie,Richcraft,UC,Canal,Nicol,Mackenzie/Minto,Frontenac,ResComms,CTTC,Robertson
    Loeb,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_LEFT,NAV_LEFT,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    Southam,NAV_PASS,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_PASS,NAV_PASS,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    TunnelJunction,NAV_RIGHT,NAV_RIGHT,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    Library,NAV_LEFT,NAV_LEFT,NAV_LEFT,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    Azrieli/Tory,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    Steacie,NAV_RIGHT,NAV_RIGHT,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_DOCK,NAV_PASS,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    Richcraft,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_DOCK,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS
    UC,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_PASS,NAV_PASS,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    Canal,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN
    Nicol,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_U-TURN,NAV_LEFT,NAV_LEFT
    Mackenzie/Minto,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_DOCK,NAV_U-TURN,NAV_U-TURN,NAV_PASS,NAV_PASS
    Frontenac,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_DOCK,NAV_U-TURN,NAV_PASS,NAV_PASS
    ResComms,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_DOCK,NAV_PASS,NAV_PASS
    CTTC,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_RIGHT,NAV_DOCK,NAV_PASS
    Robertson,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_PASS,NAV_DOCK
    """

    def decide_direction(state: NavState):
        prompt = f"""
        The robot is currently at beacon {mapping.get_beacon_name(state["current_beacon"])} and needs to go to {state["destination"]}. 
        Below is a map of the tunnels with the required navigation action to get from each beacon (rows) to each other beacon (columns). 

        {map}

        The robot can only take one action at a time, and must follow the actions in the map to navigate correctly.
        If the destination is the same as the current location, it should choose NAV_DOCK.
        What direction should it take next from the following list of options based on the current state and map of the tunnels?:
        - NAV_LEFT
        - NAV_RIGHT
        - NAV_PASS
        - NAV_U-TURN
        - NAV_DOCK
        Provide only one of these directions as the output.
        """
        node.get_logger().info("***************PROMPTING LLM FOR DIRECTION DECISION...******************")
        node.get_logger().info(f"Prompt to LLM: {prompt}")

        response = ollama.generate(
            #model='gemma3:1b',
            model='qwen2:0.5b',
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