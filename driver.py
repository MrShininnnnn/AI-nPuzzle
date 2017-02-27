#By Shining (Ning.Shi) ns2945@columbia.edu Version 20170213.1
# Import 
import sys
import time
import resource
import math
# Action & Visited Check
def move_up(node):
	up_state = node.state[:]
	position = node.state.index(0)
	if position not in range(board_n):
		up_state[position] = node.state[position - board_n]
		up_state[position - board_n] = 0
		return up_state
	else:
		pass

def move_down(node):
	down_state = node.state[:]
	position = node.state.index(0)
	if position not in range(board_n**2 - board_n, board_n**2):
		down_state[position] = node.state[position + board_n]
		down_state[position + board_n] = 0
		return down_state
	else:
		pass

def move_left(node):
	left_state = node.state[:]
	position = node.state.index(0)
	if position not in position_list_left:
		left_state[position] = node.state[position - 1]
		left_state[position - 1] = 0
		return left_state
	else:
		pass

def move_right(node):
	right_state = node.state[:]
	position = node.state.index(0)
	if position not in position_list_right:
		right_state[position] = node.state[position + 1]
		right_state[position + 1] = 0
		return right_state
	else:
		pass
# Create Node
def create_node(state, parent, path_to_goal, cost_of_path):
	return Node(state, parent, path_to_goal, cost_of_path)
class Node:
	def __init__(self, state, parent, path_to_goal, cost_of_path):
		self.state = state
		self.parent = parent
		self.path_to_goal = path_to_goal
		self.cost_of_path = cost_of_path
# Create Node for A*
def create_node_ast(state, parent, path_to_goal, cost_of_path, f):
	return Node_ast(state, parent, path_to_goal, cost_of_path, f)
class Node_ast:
	def __init__(self, state, parent, path_to_goal, cost_of_path, f):
		self.state = state
		self.parent = parent
		self.path_to_goal = path_to_goal
		self.cost_of_path = cost_of_path
		self.f = f
# Expand Node 
def expand_node(node):
	expanded_nodes = []
	expanded_nodes.append(create_node(move_up(node), node, node.path_to_goal + " Up", node.cost_of_path + 1))
	expanded_nodes.append(create_node(move_down(node), node, node.path_to_goal + " Down", node.cost_of_path + 1))
	expanded_nodes.append(create_node(move_left(node), node, node.path_to_goal + " Left", node.cost_of_path + 1))
	expanded_nodes.append(create_node(move_right(node), node, node.path_to_goal + " Right", node.cost_of_path + 1))
	expanded_nodes = filter( lambda node: node.state is not None, expanded_nodes)
	return expanded_nodes
# Expand Node for A*
def expand_node_ast(node):
	expanded_nodes = []
	expanded_nodes.append(create_node_ast(move_up(node), node, node.path_to_goal + " Up", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes.append(create_node_ast(move_down(node), node, node.path_to_goal + " Down", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes.append(create_node_ast(move_left(node), node, node.path_to_goal + " Left", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes.append(create_node_ast(move_right(node), node, node.path_to_goal + " Right", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes = filter( lambda node: node.state is not None, expanded_nodes)
	return expanded_nodes
# Expand Node Reverse
def expand_node_reverse(node):
	expanded_nodes = []
	expanded_nodes.append(create_node(move_right(node), node, node.path_to_goal + " Right", node.cost_of_path + 1))
	expanded_nodes.append(create_node(move_left(node), node, node.path_to_goal + " Left", node.cost_of_path + 1))
	expanded_nodes.append(create_node(move_down(node), node, node.path_to_goal + " Down", node.cost_of_path + 1))
	expanded_nodes.append(create_node(move_up(node), node, node.path_to_goal + " Up", node.cost_of_path + 1))
	expanded_nodes = filter( lambda node: node.state is not None, expanded_nodes)
	return expanded_nodes
# Expand Node Reverse for IDA
def expand_node_reverse_ida(node):
	expanded_nodes = []
	expanded_nodes.append(create_node_ast(move_right(node), node, node.path_to_goal + " Right", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes.append(create_node_ast(move_left(node), node, node.path_to_goal + " Left", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes.append(create_node_ast(move_down(node), node, node.path_to_goal + " Down", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes.append(create_node_ast(move_up(node), node, node.path_to_goal + " Up", node.cost_of_path + 1, mpf(node) + node.cost_of_path))
	expanded_nodes = filter( lambda node: node.state is not None, expanded_nodes)
	return expanded_nodes
# BFS
def  bfs():
	nodes_queue = []
	nodes_expanded = set()
	nodes_check = set()
	max_fringe_size = 0
	max_search_depth = 0
	# Start
	nodes_queue.append(create_node(initial_state, None, "", 0))
	nodes_check.add(str(nodes_queue[0].state))
	while len(nodes_queue) != 0: 
		# Remove the First One of the Queue
		node_next = nodes_queue.pop(0)
		nodes_expanded.add(str(node_next.state))
		nodes_check.remove(str(node_next.state))
		# Goal Check
		if node_next.state == goal_state:
			#Fringe_Size
			fringe_size = len(nodes_queue)
			#Max_Search_Depth
			for node in nodes_queue:
				if node.cost_of_path >= max_search_depth:
					max_search_depth = node.cost_of_path
			solution = [node_next, fringe_size, max_fringe_size, len(nodes_expanded) - 1, max_search_depth]
			return solution
		# Extend
		temp_queue = []
		temp_queue.extend(expand_node(node_next))
		for temp_node in temp_queue:
			# Visited Check
			if str(temp_node.state) not in nodes_expanded and str(temp_node.state) not in nodes_check:
				nodes_queue.append(temp_node)
				nodes_check.add(str(temp_node.state))
			else:
				pass
		# Max_Fringe_Size
		if len(nodes_queue) >= max_fringe_size:
			max_fringe_size = len(nodes_queue)
	return None
#DFS
def dfs():
	nodes_queue = []
	nodes_expanded = set()
	nodes_check = set()
	max_fringe_size = 0
	max_search_depth = 0
	# Start
	nodes_queue.append(create_node(initial_state, None, "", 0))
	nodes_check.add(str(nodes_queue[0].state))
	while len(nodes_queue) != 0: 
		# Remove the First One of the Stack
		node_next = nodes_queue.pop()
		nodes_expanded.add(str(node_next.state))
		nodes_check.remove(str(node_next.state))
		# Goal Check
		if node_next.state == goal_state:
			#Fringe_Size
			fringe_size = len(nodes_queue)
			#Max_Search_Depth
			if temp_node.cost_of_path >= max_search_depth:
				max_search_depth = temp_node.cost_of_path
			solution = [node_next, fringe_size, max_fringe_size, len(nodes_expanded) - 1, max_search_depth]
			return solution
		# Extend
		temp_queue = []
		temp_queue.extend(expand_node_reverse(node_next))
		for temp_node in temp_queue:
			# Visited Check
			if str(temp_node.state) not in nodes_expanded and str(temp_node.state) not in nodes_check:
				nodes_queue.append(temp_node)
				nodes_check.add(str(temp_node.state))
				#Max_Search_Depth
				if temp_node.cost_of_path >= max_search_depth:
					max_search_depth = temp_node.cost_of_path
				else:
					pass
			else:
				pass
		# Max_Fringe_Size
		if len(nodes_queue) >= max_fringe_size:
			max_fringe_size = len(nodes_queue)
	return None
#A*
def ast():
	nodes_queue = []
	nodes_expanded = set()
	nodes_check = set()
	max_fringe_size = 0
	max_search_depth = 0
	# Start
	nodes_queue.append(create_node_ast(initial_state, None, "", 0, 0))
	nodes_check.add(str(nodes_queue[0].state))
	while len(nodes_queue) != 0: 
		# Priority Queue
		nodes_queue = sorted(nodes_queue, key = lambda x: (x.f))
		# Remove the First One of the Queue
		node_next = nodes_queue.pop(0)
		nodes_expanded.add(str(node_next.state))
		nodes_check.remove(str(node_next.state))
		# Goal Check
		if node_next.state == goal_state:
			#Fringe_Size
			fringe_size = len(nodes_queue)
			#Max_Search_Depth
			for node in nodes_queue:
				if node.cost_of_path >= max_search_depth:
					max_search_depth = node.cost_of_path
			solution = [node_next, fringe_size, max_fringe_size, len(nodes_expanded) - 1, max_search_depth]
			return solution
		# Extend
		temp_queue = []
		temp_queue.extend(expand_node_ast(node_next))
		for temp_node in temp_queue:
			# Visited Check
			if str(temp_node.state) not in nodes_expanded and str(temp_node.state) not in nodes_check:
				nodes_queue.append(temp_node)
				nodes_check.add(str(temp_node.state))
			else:
				pass
		# Max_Fringe_Size
		if len(nodes_queue) >= max_fringe_size:
			max_fringe_size = len(nodes_queue)
	return None
# Manhattan Priority Function
def mpf(current_state):
	h = 0
	if current_state != None:
		for i in range(1,len(current_state.state)):
			h += abs((current_state.state.index(i) % 3 + 1) - (goal_state.index(i) % 3 +1 )) + abs(3 - (current_state.state.index(i) // 3) - (3 - (goal_state.index(i) // 3)))
		return h
	else:
		pass
# Manhattan Priority Function for IDA
def mpf_ida(current_state):
	h = 0
	for i in range(1,len(current_state)):
		h += abs((current_state.index(i) % 3 + 1) - (goal_state.index(i) % 3 +1 )) + abs(3 - (current_state.index(i) // 3) - (3 - (goal_state.index(i) // 3)))
	return h
# for IDA
def ida():
	max_fringe_size = 0
	max_search_depth = 0
	node_next_queue = set()
	f_list = set()
	f_past = set()
	# Root F
	depth_limit = mpf_ida(initial_state)
	# Loop
	while True:
		for limit in range(depth_limit + 1):
			#print limit
			solution = dls(limit, max_fringe_size, max_search_depth, node_next_queue)
			if solution != None:
				if len(solution) > 3:
					return solution
				else:
					node_next_queue = (solution[0])
					max_fringe_size = (solution[1])
			else: 
				pass
		for node in node_next_queue:
			if node.cost_of_path == depth_limit:
				f_list.add(node.f)
			else:
				pass
		if min(f_list) in f_past:
			f_list.remove(min(f_list))
		else:
			pass
		if min(f_list) > depth_limit:
			depth_limit = min(f_list)
			f_past.add(depth_limit)
		else:
			pass
	return None

def dls(depth_limit, max_fringe_size, max_search_depth, node_next_queue):
	nodes_queue = []
	nodes_expanded = set()
	nodes_check = set()
	node_next_queue = []
	# Start
	nodes_queue.append(create_node_ast(initial_state, None, "", 0, mpf_ida(initial_state)))
	nodes_check.add(str(nodes_queue[0].state))
	while len(nodes_queue) != 0: 
		# Remove the First One of the Stack 
		node_next = nodes_queue.pop()
		#print "depth_limit: %i" % (depth_limit)
		while len(nodes_queue) != 0 and node_next.cost_of_path > depth_limit:
			node_next = nodes_queue.pop()
		#print "node_next.cost_of_path: %i" % (node_next.cost_of_path)
		node_next_queue.append(node_next)
		nodes_expanded.add(str(node_next.state))
		nodes_check.remove(str(node_next.state))
		# Goal Check
		if node_next.state == goal_state:
			#Fringe_Size
			fringe_size = len(nodes_queue)
			#Max_Search_Depth
			if temp_node.cost_of_path >= max_search_depth:
				max_search_depth = temp_node.cost_of_path
				solution = [node_next, fringe_size, max_fringe_size, len(nodes_expanded) - 1, max_search_depth, node_next.f]
				return solution
			else:
				pass
		else:
			pass
		# Extend
		temp_queue = []
		temp_queue.extend(expand_node_reverse_ida(node_next))
		for temp_node in temp_queue:
			# Visited Check
			if str(temp_node.state) not in nodes_expanded and str(temp_node.state) not in nodes_check:
					nodes_queue.append(temp_node)
					nodes_check.add(str(temp_node.state))
			else:
				pass
		# Max_Fringe_Size
		if len(nodes_queue) >= max_fringe_size:
			max_fringe_size = len(nodes_queue)
	else:
		pass
	solution = []
	solution = (node_next_queue, max_fringe_size)
	return solution
# Output
def print_all(start_time, path_to_goal, cost_of_path, fringe_size, max_fringe_size, nodes_expanded, max_search_depth):
	print "path_to_goal:%s" % (path_to_goal)
	print "cost_of_path: %s" % (cost_of_path) 
	print "nodes_expanded: %i" % (nodes_expanded)
	print "fringe_size: %i" % (fringe_size)
	print "max_fringe_size: %i" % (max_fringe_size)
	print "search_depth: %i" % (cost_of_path)
	print "max_search_depth: %i" % (max_search_depth) 
	print "running_time: %s" % (time.time() - start_time)
	print "max_ram_usage: %s" % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000)
	output = open("output", "wb")
	output.write("path_to_goal:%s \n" % (path_to_goal));
	output.write("cost_of_path: %s \n" % (cost_of_path));
	output.write("nodes_expanded: %i \n" % (nodes_expanded));
	output.write("fringe_size: %i \n" % (fringe_size));
	output.write("max_fringe_size: %i \n" % (max_fringe_size));
	output.write("search_depth: %i \n" % (cost_of_path));
	output.write("max_search_depth: %i \n" % (max_search_depth));
	output.write("running_time: %s \n" % (time.time() - start_time));
	output.write("max_ram_usage: %s \n" % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1000))
	output.close()
# Go!
start_time = time.time()
initial_state = map(int, sys.argv[2].split(","))
goal_state = sorted(initial_state)
board_n = int(math.sqrt(len(goal_state)))
position_list_left = set()
position_list_right = set()
if initial_state == goal_state:
	print "The imput board is the goal"
else:
	for i in range (board_n):
		position_list_left.add(board_n*i)
	for i in range (1, board_n+1):
		position_list_right.add(board_n * i - 1)
	#Strategy
	if sys.argv[1] == "bfs": 
		solution = bfs()
	elif sys.argv[1] == "dfs":
		solution = dfs()
	elif sys.argv[1] == "ast":
		solution = ast()
	elif sys.argv[1] == "ida":
		solution = ida()
	else:
		pass
	if solution == None:
		print "There is no solution."
	else:	
		print_all(start_time, solution[0].path_to_goal, solution[0].cost_of_path, solution[1], solution[2], solution[3], solution[4])




















