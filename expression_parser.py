expr = ""
raw_filename = "raw_expr.txt"
parsed_filename = "parsed_expr.txt"

with open(raw_filename, "r") as raw_file:
    #expr = raw_file.read().replace('\n', '')
    expr = raw_file.read()

expr = expr.replace("_left_leg", "")
expr = expr.replace("g", "config.g")

    
expr = expr.replace("Ixx", "config.Ixx")
expr = expr.replace("Ixy", "config.Ixy")
expr = expr.replace("Ixz", "config.Ixz")

expr = expr.replace("Iyx", "config.Iyx")
expr = expr.replace("Iyy", "config.Iyy")
expr = expr.replace("Iyz", "config.Iyz")

expr = expr.replace("Izx", "config.Izx")
expr = expr.replace("Izy", "config.Izy")
expr = expr.replace("Izz", "config.Izz")

expr = expr.replace("m_", "config.m_")

expr = expr.replace("l1", "config.l1")
expr = expr.replace("l2", "config.l2")
expr = expr.replace("l3", "config.l3")
expr = expr.replace("l4", "config.l4")
expr = expr.replace("l5", "config.l5")
expr = expr.replace("l6", "config.l6")

expr = expr.replace("f_x", "f(0, 0)")
expr = expr.replace("f_y", "f(1, 0)")
expr = expr.replace("f_z", "f(2, 0)")

with open(parsed_filename, "w") as parsed_file:
	    parsed_file.write(expr)