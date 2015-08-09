experiment_name = "sysadmin"

log_dir = "exp_log"

agent_echo = Agent("echo run an agent")
agent_random = Agent("random_agent");
agent_null = Agent("null_agent");
#agent_spudd = Agent("glue_spudd ../experiments/agent_spudd/SPUDD-OPTDual-ma-sysadmin-ring-6.ADD", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/libcrl")
agent_alp = Agent("glue_alp ring 20");
#agent_alp1 = Agent("glue_alp ring 20 -w /home/philipp/Desktop/experiments/Coordination_Discovery/sysadmin_ring_20/data_sort/weights0.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext")
#agent_alp2 = Agent("glue_alp ring 20 -w /home/philipp/Desktop/experiments/Coordination_Discovery/sysadmin_ring_20/data_sort/weights30.txt -b /home/philipp/Desktop/experiments/Coordination_Discovery/sysadmin_ring_20/data_sort/conj30.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext")
#agent_alp3 = Agent("glue_alp ring 20 -w /home/philipp/Desktop/experiments/Coordination_Discovery/sysadmin_ring_20/data_sort/weights40.txt -b /home/philipp/Desktop/experiments/Coordination_Discovery/sysadmin_ring_20/data_sort/conj40.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext")
#agent_alp4 = Agent("glue_alp ring 20 -w /home/philipp/Desktop/experiments/Coordination_Discovery/sysadmin_ring_20/data_sort/weights70.txt -b /home/philipp/Desktop/experiments/Coordination_Discovery/sysadmin_ring_20/data_sort/conj70.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext")

env_echo = Environment("echo run an environment")
glue_sysadmin1 = Environment("glue_sysadmin ring 20")



num_steps = 30
num_trials = 50
num_runs = 10

#instance names must be unique
instances = [
    #Instance("DEBUG", agent_echo, env_echo, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RAND", agent_random, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("NULL", agent_null, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("SPUDD", agent_spudd, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("ALP", agent_alp, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("ALP1", agent_alp1, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("ALP2", agent_alp2, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("ALP3", agent_alp3, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("ALP4", agent_alp4, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
]
