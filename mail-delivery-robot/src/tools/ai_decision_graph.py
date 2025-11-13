from langgraph.graph import StateGraph, END
import ollama

class NavState(dict):
    current_beacon: str
    destination: str
    direction: str | None

def build_nav_graph():

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

        response = ollama.run(mode = "qwen2.5:0.5b",prompt = prompt)

        state["direction"] = response.strip().split()[0]

        return state
    
    graph.add_node("Decide Direction", decide_direction)
    graph.set_entry_point("Decide Direction")
    graph.add_edge("Decide Direction", END)

    return graph.compile()

