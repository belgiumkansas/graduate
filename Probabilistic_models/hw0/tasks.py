#!/usr/bin/env python2
''' Homework 0 code'''

import numpy as np
import pandas as pd
from sklearn.naive_bayes import GaussianNB

def main():

    print_cond = 0
    ''' the main task function'''
    f = open('titanic.txt', 'r')
    titanic_raw = [['Class', 'Age', 'Gender', 'Outcome']]

    while True:
        line = f.readline().split()
        if not line:
            break
        titanic_raw.append(line)
    ''' some colorfull transformation of data
    TODO make more efficient better'''
    titanic_raw = np.asarray(titanic_raw)
    df = pd.DataFrame(titanic_raw)
    df.columns = ['Class', 'Age', 'Gender', 'Outcome']
    df.drop(df.index[[0]], inplace=True)

    total_tab = pd.crosstab(index=df['Class'],
                           columns=[df['Gender'], df['Age']])

    alive_df = df.loc[df['Outcome'] == 'yes']
    alive_tab = pd.crosstab(index=alive_df['Class'],
                           columns=[alive_df['Gender'], alive_df['Age']])

    dead_df = df.loc[df['Outcome'] == 'no']
    '''make the table'''
    dead_tab = pd.crosstab(index=dead_df['Class'],
                           columns=[dead_df['Gender'], dead_df['Age']])


    # task 1
    if print_cond: print"dead"
    dead_pct_tab =  dead_tab / total_tab * 100
    if 1: print dead_pct_tab
    if print_cond: print(total_tab - dead_tab) /total_tab * 100


    class_giv_death = (dead_df.groupby("Class")["Outcome"].value_counts()
                      /dead_df.shape[0] * 100)
    if print_cond : print class_giv_death

    age_giv_death = (dead_df.groupby("Age")["Outcome"].value_counts()
                    /dead_df.shape[0] * 100)
    if print_cond: print age_giv_death

    gender_giv_death = (dead_df.groupby("Gender")["Outcome"].value_counts()
                       /dead_df.shape[0] * 100)
    if print_cond: print gender_giv_death

    class_giv_survive = (alive_df.groupby("Class")["Outcome"].value_counts()
                        /alive_df.shape[0] * 100)
    if print_cond: print class_giv_survive

    age_giv_survive = (alive_df.groupby("Age")["Outcome"].value_counts()
                      /alive_df.shape[0] * 100)
    if print_cond: print age_giv_survive

    gender_giv_survive = (alive_df.groupby("Gender")["Outcome"].value_counts()
                      /alive_df.shape[0] * 100)
    if print_cond: print gender_giv_survive

    if print_cond: print (class_giv_survive["1st"]*gender_giv_survive["female"]
                          *age_giv_survive["adult"]/ (100*100))

    if print_cond: print (class_giv_death["1st"]*gender_giv_death["female"]
                          *age_giv_death["adult"]/ (100*100))


    Pr_death = float(dead_df.shape[0])/df.shape[0]

    print "death Pr(d):", Pr_death
    for clss in ["1st", "2nd", "3rd", "crew"]:
        for age in ["adult", "child"]:
            for gender in ["male", "female"]:
                print
                label = "death" + clss+" " + age+" " + gender
                print label
                P_B_giv_A = class_giv_death[clss]/100
                P_B_giv_A *= age_giv_death[age]/100
                P_B_giv_A *= gender_giv_death[gender]/100
                top = P_B_giv_A * Pr_death

                P_B_giv_A_not = class_giv_survive[clss]/100
                P_B_giv_A_not *= age_giv_survive[age]/100
                P_B_giv_A_not *= gender_giv_survive[gender]/100
                P_B_giv_A_not *= (1 - Pr_death)

                P_B = top[0] + P_B_giv_A_not[0]

                P_A_giv_B = top[0]/P_B


                #print "Pr(B|A)",  P_B_giv_A[0]
                #print "Pr(A)", Pr_death
                #print "Pr(B|A)*Pr(A)", top[0]
                #print "P(B)", P_B
                print "Pr(A|B)", P_A_giv_B*100


    for clss in ["1st", "2nd", "3rd", "crew"]:
        for age in ["adult", "child"]:
            for gender in ["male", "female"]:
                print
                label = "survive" + clss+" " + age+" " + gender
                print label
                P_B_giv_A = class_giv_survive[clss]/100
                P_B_giv_A *= age_giv_survive[age]/100
                P_B_giv_A *= gender_giv_survive[gender]/100
                top = P_B_giv_A * (1-Pr_death)

                P_B_giv_A_not = class_giv_death[clss]/100
                P_B_giv_A_not *= age_giv_death[age]/100
                P_B_giv_A_not *= gender_giv_death[gender]/100
                P_B_giv_A_not *= ( Pr_death)

                P_B = top[0] + P_B_giv_A_not[0]


                P_A_giv_B = top[0]/P_B*100

                #print "Pr(B|A)",  P_B_giv_A[0]
                #print "Pr(A)", Pr_death
                #print "Pr(B|A)*Pr(A)", top[0]
                #print "P(B)", P_B
                print "Pr(A|B)", P_A_giv_B

    #print df.groupby("Outcome").sum()

def Pr_X1_X2_X3_giv_Y(X1, X2, X3, Y, table):
    print table

def chain(P_A_Giv_B, P_A, P_B):
    '''(P_A_Giv_B * P_A)/P_B'''
    temp = P_A_Giv_B * P_A
    temp = temp /P_B * 100
    return temp





if __name__ == '__main__':
    main()
