"""
    Simple ga server for the basicbot.
"""
 
import argparse
import json
import zmq
import random
import threading
 
from deap import base
from deap import creator
from deap import tools

class senderThread(threading.Thread):
    def __init__(self, threadID, socket, population):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.socket = socket
        self.population = population
 
    def run(self):
        print("\t\t\t\tStarting Sender Thread:"+str(self.threadID))
        self.send_data()
        print("\t\t\t\tExiting Sender Thread:"+str(self.threadID))
 
    def send_data(self):
        """ Send data to worker processes. 
 
        Args:
            socket: socket to send the data out on.
                - Persistant throughout execution for now.
        """
        for ind in self.population:
            ind_pkt = {'id':ind['id'],'genome':ind['genome'], 'fitness':-1.0}
            msg = json.dumps(ind_pkt)
            socket.send(msg)

# Process inputs.
parser = argparse.ArgumentParser()
parser.add_argument("--gens", type=int, default=100, help="Number of generations to run evolution for.")
parser.add_argument("--pop_size", type=int, default=100, help="Population size for evolution.")
parser.add_argument("--eval_time", type=float, default=10., help="Simulation time for an individual.")
parser.add_argument("--run_num", type=int, default=0, help="Run Number")
parser.add_argument("--output_path", type=str, default="./", help="Output path")
args = parser.parse_args()

# Initialize the random number seed.
random.seed(args.run_num)
 
# Setup the socket to send genome data out on.
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.bind('tcp://127.0.0.1:5000')
 
# Setup the socket to read the responses on.
receiver = context.socket(zmq.PULL)
receiver.bind('tcp://127.0.0.1:5010')
 
print("Press Enter when the workers are ready: ")
_ = raw_input()
print("Sending tasks to workers")

def format_float(value):
    """ Return a formatted float value capable of being printed. """
    return float("{0:.4f}".format(value))

def init_gene():
    """ Initialize a gene in the range of 0 to 10. """
    return format_float(random.random()*10.0)

def init_individual():
    """ Initialize an individual. """
    return {'id':0,'genome':[
                init_gene(), # center_spin_thresh
                init_gene(), # center_drive_thresh
                init_gene(), # center_stop_thresh
                init_gene() # stopping_thresh
            ], 
            'fitness':-1.0}


def mutate_value(value,low_lim,upp_lim):
    """ Mutate a value by a gaussian within the bounds.  

    Args:
        value: initial value of the parameter.
        upp_lim: upper limit of the parameter
        low_lim: lower limit of the parameter
    """
    value = format_float(random.gauss(value, (upp_lim-low_lim)*0.1)) # Mutate in the range of 10% SD of the value
    if(value > upp_lim):
        value = upp_lim
    elif(value < low_lim):
        value = low_lim
    return value

def mutate(individual, mut_prob=0.04):
    """ Mutate an individual. 

    Args:
        individual: list of floats to mutate
        mut_prob: mutation probability per element in the genome.
    """

    for i in range(len(individual)):
        if random.random() < mut_prob:
            individual[i] = mutate_value(individual[i],0.0,10.0)

    return (individual,)

def evaluate_population(population, gen):
    """ Evaluate a population and set fitnesses appropriately.

    Args:
        population: list of individuals
        gen: generation being conducted
    Returns:
        list of population.
    """
    # Start a thread to send the data.
    sendThread = senderThread(1, socket, population)
    sendThread.start()
     
    # Read the responses on the receiver socket.
    i = len(population)
    while i > 0:
        data = json.loads(receiver.recv())
        print(data['fitness'],data['id'])
        population[get_index_of_ind(population,data['id'])]['fitness'] = data['fitness']
        i -= 1
     
    # Wait for the send thread to complete.
    sendThread.join()

    return population

def get_index_of_ind(population, ind_id):
    """ Get the index of the individual in the population. """
    for i,ind in enumerate(population):
        if ind['id'] == ind_id:
            return i

# Establish name of the output files and write appropriate headers.
out_fit_file = args.output_path+str(args.run_num)+"_fitnesses.dat"
out_time_file = args.output_path+str(args.run_num)+"_timing.dat"
geneaology_file = args.output_path+str(args.run_num)+"_geneaology.dat"
#writeHeaders(out_fit_file)

# Create an individual.
creator.create("Fitness", base.Fitness, weights=(-1.0,)) # Minimize time to reach cylinder
creator.create("Individual", list, fitness=creator.Fitness)

# Create the toolbox for setting up DEAP functionality.
toolbox = base.Toolbox()

# Create the toolbox for tracking history.
history = tools.History()

# Define an individual for use in constructing the population.

# Gene generator.
toolbox.register("attr_gene",init_gene)

# Initialize the genome for the individual.
toolbox.register("individual", init_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

toolbox.register("mutate", mutate)
toolbox.register("mate", tools.cxTwoPoint)

# Decorate the variation operators
toolbox.decorate("mate", history.decorator)
toolbox.decorate("mutate", history.decorator)

# Create a population as a list.
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Register the selection function.
toolbox.register("select", tools.selTournament, tournsize=2)

# Create the Hall-of-Fame
#hof = tools.HallOfFame(maxsize=100)

# Crossover and mutation probability
cxpb, mutpb = 0.5, 0.05

# Setup the population.
pop = toolbox.population(n=args.pop_size)
# history.update(pop)

# Can conduct evaluations this way.
for p in pop:
    print(p['id'],": ",p['genome'][0],p['genome'][1],p['genome'][2],['genome']p[3])

    p['fitness'] = (random.random(),)
    print(p.fitness)

pop = evaluate_population(pop,0)

# # Log the progress of the population. (For Generation 0)
# writeGeneration(out_fit_file,0,pop)

for g in range(1,args.gens):

    # Pull out the elite individual to save for later.
    elite = tools.selBest(pop, k=1)

    offspring = toolbox.select(pop, k=len(pop)-1)
    offspring = list(map(toolbox.clone, offspring))

    # Update the Hall of Fame
    #hof.update(pop)

    for child1, child2 in zip(offspring[::2], offspring[1::2]):
        if random.random() < CXPB:
            toolbox.mate(child1, child2)
            del child1.fitness.values
            del child2.fitness.values

    for mutant in offspring:
        if random.random() < MUTPB:
            toolbox.mutate(mutant)
            del mutant.fitness.values

    pop[:] = offspring

    # Request new id's for the population.
    for i in range(len(pop)):
        pop[i]['id'] += len(pop)

    evaluate_population(pop, g)

    print("Generation "+str(g))
#     # Log the progress of the population.
#     writeGeneration(out_fit_file,g,pop)
#     #history.update(pop)
 
print("Closing Socket")
socket.close()
receiver.close()
