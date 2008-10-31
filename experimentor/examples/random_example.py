experiment_name = "random_example"

log_dir = "/home/jasmuth/exp_log"

agent_1 = Agent("random_agent")
agent_2 = Agent("random_agent")

env = Environment("walk_env")

#instance names must be unique
instances = [
    Instance("Random Walk 1", agent_1, env, trials=100, runs=4),
    Instance("Random Walk 2", agent_2, env, trials=100, runs=4)
]
