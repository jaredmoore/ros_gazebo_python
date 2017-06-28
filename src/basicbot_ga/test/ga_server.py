"""
    Simple ga server for the basicbot.
"""
 
import argparse
# import json
# import zmq
 
import random
#import threading
 
from deap import base
from deap import creator
from deap import tools

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
 
# # Setup the socket to send genome data out on.
# context = zmq.Context()
# socket = context.socket(zmq.PUSH)
# socket.bind('tcp://127.0.0.1:5000')

# # Setup the socket to send genomes to the distributer thread on.
# # genome_context = zmq.Context()
# # genome_socket = genome_context.socket(zmq.PUSH)
# # genome_socket.bind('tcp://127.0.0.1:5005')
 
# # Setup the socket to read the responses on.
# receiver = context.socket(zmq.PULL)
# receiver.bind('tcp://127.0.0.1:5010')
 
# print("Press Enter when the workers are ready: ")
# _ = raw_input()
# print("Sending tasks to workers")






def format_float(value):
    """ Return a formatted float value capable of being printed. """
    return float("{0:.4f}".format(value))

def init_gene():
    """ Initialize a gene in the range of 0 to 10. """
    return format_float(random.random()*10.0)


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
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_gene, 4)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)


toolbox.register("mutate", mutate)
toolbox.register("mate", tools.cxTwoPoint)

# Decorate the variation operators
toolbox.decorate("mate", history.decorator)
toolbox.decorate("mutate", history.decorator)

# Create a population as a list.
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Register the evaluation function.
# TODO: Redo evaluation function.
# toolbox.register("evaluate", evaluate_individual)

# Register the selection function.
toolbox.register("select", tools.selTournament, tournsize=2)

# Create the Hall-of-Fame
#hof = tools.HallOfFame(maxsize=100)

# Crossover and mutation probability
cxpb, mutpb = 0.5, 0.05

# Setup the population.
pop = toolbox.population(n=args.pop_size)
# history.update(pop)

for p in pop:
    print(p)

# # Run the first set of evaluations.
# fitnesses = toolbox.map(toolbox.evaluate, pop)
# for ind, fit in zip(pop, fitnesses):
#     ind.fitness.values = fit

# # Log the progress of the population. (For Generation 0)
# writeGeneration(out_fit_file,0,pop)

# # writeTimeInformationHeaders(out_time_file)

# for g in range(1,args.gens):
#     # select_time = time.time()
#     # Pull out the elite individual to save for later.
#     elite = tools.selBest(pop, k=1)

#     pop = toolbox.select(pop, k=len(pop)-1)
#     pop = [toolbox.clone(ind) for ind in pop]
#     # select_time = time.time() - select_time

#     # Update the Hall of Fame
#     #hof.update(pop)

#     # id_time = time.time()
#     # Request new id's for the population.
#     for ind in pop:
#         ind.get_new_id()
#     # id_time = time.time() - id_time


#     # cross_time = time.time()
#     for child1, child2 in zip(pop[::2], pop[1::2]):
#         if random.random() < cxpb:
#             # Must serialize and deserialize due to the type of object.
#             # child1_serialized, child2_serialized = toolbox.mate(child1.serialize(), child2.serialize())
#             child1, child2 = toolbox.mate(child1, child2)
#             #child1.deserialize(child1_serialized)
#             #child2.deserialize(child2_serialized)
#             del child1.fitness.values, child2.fitness.values
#     # cross_time = time.time() - cross_time

#     # mut_time = time.time()
#     #for mutant in pop:
#     #    toolbox.mutate(mutant)
#     #    del mutant.fitness.values
#     for i in range(len(pop)):
#     #for ind in pop:
#        pop[i] = toolbox.mutate(pop[i])[0]
#        del pop[i].fitness.values
#     # mut_time = time.time() - mut_time

#     # eval_time = time.time()
#     invalids = [ind for ind in pop if not ind.fitness.valid]
#     fitnesses = toolbox.map(toolbox.evaluate, invalids)
#     for ind, fit in zip(invalids, fitnesses):
#         ind.fitness.values = fit
#     # eval_time = time.time() - eval_time

#     # Check to see if we have a new elite individual.
#     new_elite = tools.selBest(pop, k=1)
#     elite = tools.selBest([elite[0],new_elite[0]],k=1)

#     # Add the elite individual back into the population.
#     pop = elite+pop

#     print("Generation "+str(g))
#     # Log the progress of the population.
#     writeGeneration(out_fit_file,g,pop)
#     #history.update(pop)

# writeGeneaology(geneaology_file,history.genealogy_tree)
# # Write the hall of fame out to a file.
# #writeHOF(out_hof_file,hof)

#     # Log the timing of the run.
#     # writeTimeInformation(out_time_file,g,select_time, id_time, cross_time, mut_time, eval_time)

























 
# # Read the responses on the receiver socket.
# i = 
# while i > 0:
#     data = json.loads(receiver.recv())
#     print(data['fitness'],data['id'])
#     i -= 1
 
# print("Closing Socket")
# socket.close()
# receiver.close()
