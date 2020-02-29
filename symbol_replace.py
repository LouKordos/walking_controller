expr = ""
expression_is_python = False

with open('expression.txt', 'r') as myfile:
    expr = myfile.read().replace('\n', '')

expr = expr.replace("(t)", "")

if expression_is_python is False:

    expr = expr.replace("m_{hip3}", "m_hip3")
    expr = expr.replace("m_{hip2}", "m_hip2")
    expr = expr.replace("m_{hip1}", "m_hip1")

    expr = expr.replace("m_{ul}", "m_ul")
    expr = expr.replace("m_{ll}", "m_ll")
    expr = expr.replace("m_{foot}", "m_foot")

    expr = expr.replace("l1_{CoM}", "l1_CoM")
    expr = expr.replace("l2_{CoM}", "l2_CoM")
    expr = expr.replace("l3_{CoM}", "l3_CoM")
    expr = expr.replace("l4_{CoM}", "l4_CoM")
    expr = expr.replace("l5_{CoM}", "l5_CoM")
    expr = expr.replace("l6_{CoM}", "l6_CoM")

    expr = expr.replace("l_1", "l1")
    expr = expr.replace("l_2", "l2")
    expr = expr.replace("l_3", "l3")
    expr = expr.replace("l_4", "l4")
    expr = expr.replace("l_5", "l5")
    expr = expr.replace("l_6", "l6")

    expr = expr.replace("Ixx_{hip3}", "Ixx_hip3")
    expr = expr.replace("Ixx_{hip2}", "Ixx_hip2")
    expr = expr.replace("Ixx_{hip1}", "Ixx_hip1")
    expr = expr.replace("Ixx_{ul}", "Ixx_ul")
    expr = expr.replace("Ixx_{ll}", "Ixx_ll")
    expr = expr.replace("Ixx_{foot}", "Ixx_foot")

    expr = expr.replace("Ixy_{hip3}", "Ixy_hip3")
    expr = expr.replace("Ixy_{hip2}", "Ixy_hip2")
    expr = expr.replace("Ixy_{hip1}", "Ixy_hip1")
    expr = expr.replace("Ixy_{ul}", "Ixy_ul")
    expr = expr.replace("Ixy_{ll}", "Ixy_ll")
    expr = expr.replace("Ixy_{foot}", "Ixy_foot")

    expr = expr.replace("Ixz_{hip3}", "Ixz_hip3")
    expr = expr.replace("Ixz_{hip2}", "Ixz_hip2")
    expr = expr.replace("Ixz_{hip1}", "Ixz_hip1")
    expr = expr.replace("Ixz_{ul}", "Ixz_ul")
    expr = expr.replace("Ixz_{ll}", "Ixz_ll")
    expr = expr.replace("Ixz_{foot}", "Ixz_foot")

    expr = expr.replace("Iyx_{hip3}", "Iyx_hip3")
    expr = expr.replace("Iyx_{hip2}", "Iyx_hip2")
    expr = expr.replace("Iyx_{hip1}", "Iyx_hip1")
    expr = expr.replace("Iyx_{ul}", "Iyx_ul")
    expr = expr.replace("Iyx_{ll}", "Iyx_ll")
    expr = expr.replace("Iyx_{foot}", "Iyx_foot")

    expr = expr.replace("Iyy_{hip3}", "Iyy_hip3")
    expr = expr.replace("Iyy_{hip2}", "Iyy_hip2")
    expr = expr.replace("Iyy_{hip1}", "Iyy_hip1")
    expr = expr.replace("Iyy_{ul}", "Iyy_ul")
    expr = expr.replace("Iyy_{ll}", "Iyy_ll")
    expr = expr.replace("Iyy_{foot}", "Iyy_foot")

    expr = expr.replace("Iyz_{hip3}", "Iyz_hip3")
    expr = expr.replace("Iyz_{hip2}", "Iyz_hip2")
    expr = expr.replace("Iyz_{hip1}", "Iyz_hip1")
    expr = expr.replace("Iyz_{ul}", "Iyz_ul")
    expr = expr.replace("Iyz_{ll}", "Iyz_ll")
    expr = expr.replace("Iyz_{foot}", "Iyz_foot")

    expr = expr.replace("Izx_{hip3}", "Izx_hip3")
    expr = expr.replace("Izx_{hip2}", "Izx_hip2")
    expr = expr.replace("Izx_{hip1}", "Izx_hip1")
    expr = expr.replace("Izx_{ul}", "Izx_ul")
    expr = expr.replace("Izx_{ll}", "Izx_ll")
    expr = expr.replace("Izx_{foot}", "Izx_foot")

    expr = expr.replace("Izy_{hip3}", "Izy_hip3")
    expr = expr.replace("Izy_{hip2}", "Izy_hip2")
    expr = expr.replace("Izy_{hip1}", "Izy_hip1")
    expr = expr.replace("Izy_{ul}", "Izy_ul")
    expr = expr.replace("Izy_{ll}", "Izy_ll")
    expr = expr.replace("Izy_{foot}", "Izy_foot")

    expr = expr.replace("Izz_{hip3}", "Izz_hip3")
    expr = expr.replace("Izz_{hip2}", "Izz_hip2")
    expr = expr.replace("Izz_{hip1}", "Izz_hip1")
    expr = expr.replace("Izz_{ul}", "Izz_ul")
    expr = expr.replace("Izz_{ll}", "Izz_ll")
    expr = expr.replace("Izz_{foot}", "Izz_foot")

else:
    expr = expr.replace("m_{hip3}", "m_hip3")
    expr = expr.replace("m_{hip2}", "m_hip2")
    expr = expr.replace("m_{hip1}", "m_hip1")

    expr = expr.replace("m_{ul}", "m_upper_leg")
    expr = expr.replace("m_{ll}", "m_lower_leg")
    expr = expr.replace("m_{foot}", "m_foot")

    expr = expr.replace("l1_{CoM}", "l1_com")
    expr = expr.replace("l2_{CoM}", "l2_com")
    expr = expr.replace("l3_{CoM}", "l3_com")
    expr = expr.replace("l4_{CoM}", "l4_com")
    expr = expr.replace("l5_{CoM}", "l5_com")
    expr = expr.replace("l6_{CoM}", "l6_com")

    expr = expr.replace("l_1", "l1")
    expr = expr.replace("l_2", "l2")
    expr = expr.replace("l_3", "l3")
    expr = expr.replace("l_4", "l4")
    expr = expr.replace("l_5", "l5")
    expr = expr.replace("l_6", "l6")

    expr = expr.replace("Ixx_{hip3}", "Ixx_hip3")
    expr = expr.replace("Ixx_{hip2}", "Ixx_hip2")
    expr = expr.replace("Ixx_{hip1}", "Ixx_hip1")
    expr = expr.replace("Ixx_{ul}", "Ixx_upper_leg")
    expr = expr.replace("Ixx_{ll}", "Ixx_lower_leg")
    expr = expr.replace("Ixx_{foot}", "Ixx_foot")

    expr = expr.replace("Ixy_{hip3}", "Ixy_hip3")
    expr = expr.replace("Ixy_{hip2}", "Ixy_hip2")
    expr = expr.replace("Ixy_{hip1}", "Ixy_hip1")
    expr = expr.replace("Ixy_{ul}", "Ixy_upper_leg")
    expr = expr.replace("Ixy_{ll}", "Ixy_lower_leg")
    expr = expr.replace("Ixy_{foot}", "Ixy_foot")

    expr = expr.replace("Ixz_{hip3}", "Ixz_hip3")
    expr = expr.replace("Ixz_{hip2}", "Ixz_hip2")
    expr = expr.replace("Ixz_{hip1}", "Ixz_hip1")
    expr = expr.replace("Ixz_{ul}", "Ixz_upper_leg")
    expr = expr.replace("Ixz_{ll}", "Ixz_lower_leg")
    expr = expr.replace("Ixz_{foot}", "Ixz_foot")

    expr = expr.replace("Iyx_{hip3}", "Iyx_hip3")
    expr = expr.replace("Iyx_{hip2}", "Iyx_hip2")
    expr = expr.replace("Iyx_{hip1}", "Iyx_hip1")
    expr = expr.replace("Iyx_{ul}", "Iyx_upper_leg")
    expr = expr.replace("Iyx_{ll}", "Iyx_lower_leg")
    expr = expr.replace("Iyx_{foot}", "Iyx_foot")

    expr = expr.replace("Iyy_{hip3}", "Iyy_hip3")
    expr = expr.replace("Iyy_{hip2}", "Iyy_hip2")
    expr = expr.replace("Iyy_{hip1}", "Iyy_hip1")
    expr = expr.replace("Iyy_{ul}", "Iyy_upper_leg")
    expr = expr.replace("Iyy_{ll}", "Iyy_lower_leg")
    expr = expr.replace("Iyy_{foot}", "Iyy_foot")

    expr = expr.replace("Iyz_{hip3}", "Iyz_hip3")
    expr = expr.replace("Iyz_{hip2}", "Iyz_hip2")
    expr = expr.replace("Iyz_{hip1}", "Iyz_hip1")
    expr = expr.replace("Iyz_{ul}", "Iyz_upper_leg")
    expr = expr.replace("Iyz_{ll}", "Iyz_lower_leg")
    expr = expr.replace("Iyz_{foot}", "Iyz_foot")

    expr = expr.replace("Izx_{hip3}", "Izx_hip3")
    expr = expr.replace("Izx_{hip2}", "Izx_hip2")
    expr = expr.replace("Izx_{hip1}", "Izx_hip1")
    expr = expr.replace("Izx_{ul}", "Izx_upper_leg")
    expr = expr.replace("Izx_{ll}", "Izx_lower_leg")
    expr = expr.replace("Izx_{foot}", "Izx_foot")

    expr = expr.replace("Izy_{hip3}", "Izy_hip3")
    expr = expr.replace("Izy_{hip2}", "Izy_hip2")
    expr = expr.replace("Izy_{hip1}", "Izy_hip1")
    expr = expr.replace("Izy_{ul}", "Izy_upper_leg")
    expr = expr.replace("Izy_{ll}", "Izy_lower_leg")
    expr = expr.replace("Izy_{foot}", "Izy_foot")

    expr = expr.replace("Izz_{hip3}", "Izz_hip3")
    expr = expr.replace("Izz_{hip2}", "Izz_hip2")
    expr = expr.replace("Izz_{hip1}", "Izz_hip1")
    expr = expr.replace("Izz_{ul}", "Izz_upper_leg")
    expr = expr.replace("Izz_{ll}", "Izz_lower_leg")
    expr = expr.replace("Izz_{foot}", "Izz_foot")

expr = expr.replace("cosl", "cos")
expr = expr.replace("sinl", "sin")
expr = expr.replace("powl", "pow")
expr = expr.replace(";", ";\n")

expr = expr.replace("C[0]", "double C_1")
expr = expr.replace("C[1]", "double C_2")
expr = expr.replace("C[2]", "double C_3")
expr = expr.replace("C[3]", "double C_4")
expr = expr.replace("C[4]", "double C_5")
expr = expr.replace("C[5]", "double C_6")
expr = expr.replace("C[6]", "double C_7")
expr = expr.replace("C[7]", "double C_8")
expr = expr.replace("C[8]", "double C_9")
expr = expr.replace("C[9]", "double C_10")
expr = expr.replace("C[10", "double C_11")
expr = expr.replace("C[11]", "double C_12")
expr = expr.replace("C[12]", "double C_13")
expr = expr.replace("C[13]", "double C_14")
expr = expr.replace("C[14]", "double C_15")
expr = expr.replace("C[15]", "double C_16")
expr = expr.replace("C[16]", "double C_17")
expr = expr.replace("C[17]", "double C_18")
expr = expr.replace("C[18]", "double C_19")
expr = expr.replace("C[19]", "double C_20")
expr = expr.replace("C[20]", "double C_21")
expr = expr.replace("C[21]", "double C_22")
expr = expr.replace("C[22]", "double C_23")
expr = expr.replace("C[23]", "double C_24")
expr = expr.replace("C[24]", "double C_25")

expr = expr.replace("B[0]", "double B_1")
expr = expr.replace("B[1]", "double B_2")
expr = expr.replace("B[2]", "double B_3")
expr = expr.replace("B[3]", "double B_4")
expr = expr.replace("B[4]", "double B_5")
expr = expr.replace("B[5]", "double B_6")
expr = expr.replace("B[6]", "double B_7")
expr = expr.replace("B[7]", "double B_8")
expr = expr.replace("B[8]", "double B_9")
expr = expr.replace("B[9]", "double B_10")
expr = expr.replace("B[10", "double B_11")
expr = expr.replace("B[11]", "double B_12")
expr = expr.replace("B[12]", "double B_13")
expr = expr.replace("B[13]", "double B_14")
expr = expr.replace("B[14]", "double B_15")
expr = expr.replace("B[15]", "double B_16")
expr = expr.replace("B[16]", "double B_17")
expr = expr.replace("B[17]", "double B_18")
expr = expr.replace("B[18]", "double B_19")
expr = expr.replace("B[19]", "double B_20")
expr = expr.replace("B[20]", "double B_21")
expr = expr.replace("B[21]", "double B_22")
expr = expr.replace("B[22]", "double B_23")
expr = expr.replace("B[23]", "double B_24")
expr = expr.replace("B[24]", "double B_25")



with open("expression_parsed.txt", "w") as text_file:
    text_file.write(expr)