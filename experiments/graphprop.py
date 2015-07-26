experiment_name = "graphprop"

log_dir = "exp_log"

agent_echo = Agent("echo run an agent")
agent_random = Agent("random_agent");
agent_null = Agent("null_agent");
agent_copystate = Agent("copystate_agent");
#agent_bigalp = Agent("glue_bigalp cfgs/default.xml data/sdmia/Rand50-15.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop");
agent_bigalp2 = Agent("glue_bigalp cfgs/default.xml data/sdmia/Rand50-15.txt data/agents50.sol.onlybarrier", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop");

env_echo = Environment("echo run an environment")
glue_graphprop1 = Environment("glue_graphprop cfgs/default.xml data/sdmia/Rand50-15.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop")
#glue_graphprop1_viz = Environment("glue_graphprop cfgs/default.xml data/sdmia/Rand50-15.txt --enable-stdout | ./bin/gvizz.py cfgs/default.xml data/sdmia/Rand50-15.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop")


num_steps = 200
num_trials = 50
num_runs = 50

#instance names must be unique
instances = [
    #Instance("DEBUG", agent_echo, env_echo, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RAND", agent_random, glue_graphprop1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("NULL", agent_null, glue_graphprop1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("COPYSTATE", agent_copystate, glue_graphprop1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BIGALP", agent_bigalp, glue_graphprop1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("BIGALP2", agent_bigalp2, glue_graphprop1, steps=num_steps, trials=num_trials, runs=num_runs),
]
