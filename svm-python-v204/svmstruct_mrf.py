"""A module for SVM^python for multiclass MRF learning."""

# Thomas Finley, tfinley@gmail.com

import sys
from numpy import random
import time
from operator import concat
import svmapi, array
import commands
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy.sparse import lil_matrix
from scipy.sparse import csr_matrix
from numpy.ma.core import zeros
import glpk
from bitarray import bitarray
from graphcut import *


global NUM_CLASSES
global ITER
global LP_LIST
global LOSS_WEIGHTS
global CLASSIFY_METHOD
global LEARN_METHOD
global ATTRIBUTE_METHOD
global LOSS_METHOD
global OBJECT_MAP_FILE
global ATTRIBUTE_MAP_FILE
LP_LIST= []
ITER = 0
NUM_CLASSES = 0
# default options
LOSS_METHOD = "micro"
LEARN_METHOD = "objassoc"
OBJECT_MAP_FILE = "/opt/ros/unstable/stacks/svm-python-v204/objectMap.txt"
CLASSIFY_METHOD  = "sum1.IP"
ATTRIBUTE_METHOD = "false"
ATTRIBUTE_MAP_FILE = "/opt/ros/unstable/stacks/svm-python-v204/attributeMap.txt"

def get_C_matrix(num_node_feats, num_edge_feats, num_ass_edge_feats, K ):
    num_nonass_edge_feats = num_edge_feats - num_ass_edge_feats;
    num_ones = num_node_feats*K + num_ass_edge_feats*K + num_nonass_edge_feats*K*K;
    crow = zeros(num_ones)
    ccol = zeros(num_ones)
    cval = ones(num_ones)

    index = 0;
    for l in xrange(0,num_node_feats*K):
        crow[index] = l
        ccol[index] = l
        index = index + 1

    for l in xrange(0,K):
        for i in xrange(0,num_ass_edge_feats):
            crow[index] = (num_ass_edge_feats*l + i) + num_node_feats*K
            ccol[index] = (num_edge_feats*(l*K+l) + i) +  num_node_feats*K
            index += 1

    for l in xrange(0,K):
        for k in xrange(0, K):
            for i in xrange(0,num_nonass_edge_feats):
                crow[index] =   num_node_feats*K+ (num_ass_edge_feats*K + num_nonass_edge_feats*(l*K+k)+i )
                ccol[index] =   num_node_feats*K+ (num_edge_feats*(l*K+k) + num_ass_edge_feats + i)
                index +=1


    C = csr_matrix((cval,(crow,ccol)),shape=(num_node_feats*K + num_ass_edge_feats*K + num_nonass_edge_feats*K*K , num_node_feats*K + (num_edge_feats*K*K)),dtype='d')
    return C

def get_C_obj_matrix(num_node_feats, num_edge_feats, num_ass_edge_feats, K, objMapList ):

    partialSums = [0]

    num_ass_terms = 0;
    for l in objMapList:
        num_ass_terms += len(l)*len(l)
        partialSums.append(num_ass_terms)

    num_nonass_edge_feats = num_edge_feats - num_ass_edge_feats;
    num_ones = num_node_feats*K + num_ass_edge_feats*num_ass_terms + num_nonass_edge_feats*K*K;
    crow = zeros(num_ones)
    ccol = zeros(num_ones)
    cval = ones(num_ones)

    index = 0;
    for l in xrange(0,num_node_feats*K):
        crow[index] = l
        ccol[index] = l
        index = index + 1
    
    objCnt = -1
    for o in objMapList:
        objCnt +=1;
        num_parts = len(o)
        for pl in xrange(0,num_parts):
            for pk in xrange(0,num_parts):
                for i in xrange(0,num_ass_edge_feats):
                    crow[index] = num_ass_edge_feats*(partialSums[objCnt] + pl*num_parts + pk) + i + num_node_feats*K
                    #print pl , pl , num_parts, partialSums[objCnt], objCnt, crow[index]
                    assert crow[index] < num_ones
                    
                    ccol[index] = num_edge_feats*( (int(o[pl])-1)*K+ (int(o[pk])-1) ) + i +  num_node_feats*K
                    index += 1

    for l in xrange(0,K):
        for k in xrange(0, K):
            for i in xrange(0,num_nonass_edge_feats):
                crow[index] =   num_node_feats*K+ (num_ass_edge_feats*num_ass_terms + num_nonass_edge_feats*(l*K+k)+i )
                
                assert crow[index] < num_ones
                ccol[index] =   num_node_feats*K+ (num_edge_feats*(l*K+k) + num_ass_edge_feats + i)
                index +=1

    

    C = csr_matrix((cval,(crow,ccol)),shape=(num_node_feats*K + num_ass_edge_feats*num_ass_terms + num_nonass_edge_feats*K*K , num_node_feats*K + (num_edge_feats*K*K)),dtype='d')
    return C

def parse_parameters(sparm):
    temp_arg_list = sparm.argv
    print sparm.argv
    global LEARN_METHOD
    global LOSS_METHOD
    global OBJECT_MAP_FILE
    global ATTRIBUTE_METHOD
    global ATTRIBUTE_MAP_FILE
    # set default values
    LOSS_METHOD = "micro"
    LEARN_METHOD = "objassoc"
    OBJECT_MAP_FILE = "/opt/ros/unstable/stacks/svm-python-v204/objectMap.txt"
    for i in xrange(0,len(sparm.argv)/2):
        #print i,  len(sparm.argv)/2
        opt = temp_arg_list.pop(0)
        val = temp_arg_list.pop(0)
        if(opt == "--l"):
            LOSS_METHOD = val
        if(opt == "--lm"):
            #print "setting lm to ", val
            LEARN_METHOD = val
        if(opt == "--am"):
            print "setting am to ", val
            ATTRIBUTE_METHOD = val
        if(opt == "--omf"):
            OBJECT_MAP_FILE = val
        if(opt == "--amf"):
            ATTRIBUTE_MAP_FILE = val


def parse_parameters_classify(attribute, value):
    
    global CLASSIFY_METHOD
    global LEARN_METHOD
    global LOSS_METHOD
    global OBJECT_MAP_FILE
    global ATTRIBUTE_METHOD
    global ATTRIBUTE_MAP_FILE
    # set default values
    #print attribute, value
    if(attribute == "--l"):
        LOSS_METHOD = value
    if(attribute == "--lm"):
        LEARN_METHOD = value
        #print "setting lm to ", LEARN_METHOD
    if(attribute == "--omf"):
        OBJECT_MAP_FILE = value
    if(attribute == "--amf"):
        ATTRIBUTE_MAP_FILE = value
    if(attribute == "--cm"):
        CLASSIFY_METHOD= value
    if(attribute == "--am"):
        ATTRIBUTE_METHOD= value


def read_examples(filename,sparm):
    
    print commands.getoutput('git log | head')
    print commands.getoutput('git diff --color')

    global NUM_CLASSES
    global LOSS_WEIGHTS
    global LEARN_METHOD
    global OBJECT_MAP_FILE
    global ATTRIBUTE_METHOD
    print sparm
    print LEARN_METHOD
    # Helper function for reading from files.
    def line_reader(lines):
        # returns only non-empty lines
        # strips comments (anything after '#')
        for l in lines:
            i = l.find('#')
            if i == -1: yield l.strip()

    ################
    # read the object label map file

    objMapList = [line.split() for line in line_reader(file(OBJECT_MAP_FILE))]
    #print objMapList
    #################

    ################
    # read the attribute label map file
    attrMapList=[]
    if(ATTRIBUTE_METHOD == "true"):
        print "reading attrmap"
        attrMapList = [line.split() for line in line_reader(file(ATTRIBUTE_MAP_FILE))]

    #find max attribute
    max_attribute = 0;
    for attrL in attrMapList:
        for a in attrL:
            if(int(a) > max_attribute):
                max_attribute = int(a);

    print "max_attribute = ", max_attribute
    print attrMapList
    #################


    """Parses an input file into an example sequence."""
    # This reads example files of the type read by SVM^multiclass.
    examples = []
    max_target=0
    num_node_feats=0
    num_edge_feats=0
    num_ass_edge_feats = 0

    start = time.clock()

    # Open the file and read each example.
    for input_file in file(filename):
        print input_file
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N = int(input[0][0].strip());
        E = int(input[0][1].strip());
        K = int(input[0][2].strip()); 
        if (LEARN_METHOD == "objassoc"):
            num_ass_edge_feats =  int(input[0][3].strip());
        # find the max class and number of node features -- will work for sparse representation
        for i in xrange(0,N):
            target = int(input[i+1][0]);

            if (max_target<int(target)):
                max_target=int(target)
            tokens = [line.split(':') for line in input[i+1][2:]]
            for k,v in tokens:
                if(num_node_feats<int(k)):
                    num_node_feats=int(k)
        for i in xrange(N,N+E):
            tokens = [line.split(':') for line in input[i+1][4:]]
            for k,v in tokens:
                if(num_edge_feats<int(k)):
                    num_edge_feats=int(k)
    max_target = K # use the max number of classes read from the file
    if(ATTRIBUTE_METHOD == "true"):
        max_target = max_attribute
        K = max_attribute
    print 'number of classes: ', max_target
    print 'number of node features: ', num_node_feats
    print 'number of edge features: ',num_edge_feats
    print 'number of associative features: ',num_ass_edge_feats

    print 'time spent in preprocessing data files', start-time.clock()
    
    # computing C matrix
    if (LEARN_METHOD == "objassoc"):
        C = get_C_obj_matrix(num_node_feats, num_edge_feats, num_ass_edge_feats, K, objMapList)
        #savetxt('C.txt',C.todense(),fmt='%d');
        
    class_counts = zeros(K)

    example_num=-1
    for input_file in file(filename):
        example_num+=1
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N = int(input[0][0].strip());
        E = int(input[0][1].strip());
        xrow = zeros(N*max_target * num_node_feats+E*max_target*max_target*num_edge_feats)
        xcol = zeros(N*max_target * num_node_feats+E*max_target*max_target*num_edge_feats)
        xval = zeros(N*max_target * num_node_feats+E*max_target*max_target*num_edge_feats)
        #Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
        Yn= mat(zeros((max_target*N,1)))

        #XY_test = mat(zeros((max_target*num_node_feats + max_target*max_target*num_edge_feats,1)))
        node_map = {}
        edges = mat(zeros((E,2)))
        for i in xrange(0,N):
            target = int(input[i+1][0]);
            if(ATTRIBUTE_METHOD == "true"):
                for attr in attrMapList[target-1]:
                    print target,attr
                    class_counts[int(attr)-1]+=1
                    Yn[i*max_target+(int(attr)-1),0]=1
            else:
                if(target != -1): # if the segment is unlabeled all entries of Yn for the label will be 0
                    class_counts[target-1]+=1
                    Yn[i*max_target+(target-1),0]=1
                    
            # get the segment number
            node_map[int(input[i+1][1])] = i

            # Get the features.
            tokens = [line.split(':') for line in input[i+1][2:]]
            ##print [float(v) for k,v in tokens]
            # assumes all features are present
            features = mat([float(v) for k,v in tokens])
            #print features
            f = features.transpose()
            #XY_test[(target-1)*num_node_feats:target*num_node_feats,0] += f
            # fill in the Xn matrix
            for j in xrange(0,max_target):
                ##print X[j*9:(j+1)*9,j];
                #Xn[j*num_node_feats:(j+1)*num_node_feats,i*max_target+(j)] = f.copy();
                #print features.A
                #print num_node_feats
                #print (i*(j+1)*num_node_feats) - (i*j*num_node_feats)
                xval[(i*max_target+j)*num_node_feats:(i*max_target+j+1)*num_node_feats] = features.A[0];
                for v in xrange(0,num_node_feats):
                  xrow[(i*max_target+j)*num_node_feats+v] = j*num_node_feats+v
                xcol[(i*max_target+j)*num_node_feats:(i*max_target+j+1)*num_node_feats] = i*max_target+(j)
            ##print X
        #Xe = mat(zeros((max_target*max_target*num_edge_feats,max_target*max_target*E)))
        Ye = mat(zeros((max_target*max_target*E,1)))
        for i in xrange(N,N+E):
            if(ATTRIBUTE_METHOD == "true"):
                target1 = int(input[i+1][0]);
                target2 = int(input[i+1][1]);
                for attr1 in attrMapList[target1-1]:
                    for attr2 in attrMapList[target2-1]:
                        Ye[(i-N)*max_target*max_target + (int(attr1)-1)*max_target+(int(attr2)-1)]=1

            else:
                target1 = int(input[i+1][0]);
                target2 = int(input[i+1][1]);
                if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                    Ye[(i-N)*max_target*max_target + (target1-1)*max_target+(target2-1)]=1
            # get the segment numbers
            edges[i-N,0]= node_map[int(input[i+1][2])]
            edges[i-N,1]= node_map[int(input[i+1][3])]

            tokens = [line.split(':') for line in input[i+1][4:]]
            features = mat([float(v) for k,v in tokens])
            #print features
            f = features.transpose()
            z = max_target*num_node_feats + (target1-1)*max_target*num_edge_feats+(target2-1)*num_edge_feats
            #XY_test[z : z+num_edge_feats ,0] += f
            # fill in the Xn matrix
            for j in xrange(0,max_target*max_target):
                ##print X[j*9:(j+1)*9,j];
                #Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();
                xval[N*max_target*num_node_feats + ((i-N)*max_target*max_target+j)*num_edge_feats: N*max_target*num_node_feats + ((i-N)*max_target*max_target+(j+1))*num_edge_feats] = features.copy();
                for v in xrange(0,num_edge_feats):
                  xrow[N*max_target*num_node_feats+((i-N)*max_target*max_target+j)*num_edge_feats+v] = max_target*num_node_feats+ j*num_edge_feats+v
                xcol[N*max_target*num_node_feats + ((i-N)*max_target*max_target+j)*num_edge_feats: N*max_target*num_node_feats + ((i-N)*max_target*max_target+(j+1))*num_edge_feats] = N*max_target +(i-N)*max_target*max_target+j
        #print Xn.shape[0], num_node_feats*K
        #print Xn.shape[1] , N*K
        #print Xe.shape[0], num_edge_feats*K*K
        #print Xe.shape[1] , E*K*K
        #print xval
        #print xrow
        #print xcol
        X_sparse = csr_matrix((xval,(xrow,xcol)),shape=(num_node_feats*K+num_edge_feats*K*K,N*K+(E*K*K)),dtype='d');
        #a = concatenate ((Xn, mat(zeros((Xn.shape[0],Xe.shape[1])))),1)
        #b = concatenate ((mat(zeros((Xe.shape[0],Xn.shape[1]))),Xe),1)
        #X = concatenate ((a,b))
        Y = concatenate ((Yn,Ye))
        #X_s = csr_matrix(X,dtype='d')
        #print sum(X_s.todense() - X_sparse.todense())
        #assert (sum(X_s.todense() - X_sparse.todense()) == 0)
        #assert ((X_s - X_sparse).sum() == 0)

        Y_s = csr_matrix(Y,dtype='d')
        K = max_target
        row = zeros(N*K+E*K*K)
        cols = zeros(N*K+E*K*K)
        values = ones(N*K+E*K*K)
        for i in xrange(0,N*K):
            row[i] = i
            cols[i] = i

        Yec = mat(zeros((max_target*max_target*E/2,1)))
        count = 0
        ijlk = zeros((E*K*K/2,4))
        for e in xrange(0,E):
            i = edges[e,0]
            j = edges[e,1]
            assert(i!=j)
            for l in xrange(0,K):
                start=l
                if(i>j):
                    start=l+1
                for k in xrange(start,K):
                    row[2*count+N*K] = N*K + e*K*K + l*K +k
                    cols[2*count+N*K] = count+N*K
                    redge = get_index(edges,j,i)
                    row[2*count+1+N*K] = N*K + redge*K*K + k*K +l
                    cols[2*count+1+N*K] = count+N*K
                    ijlk[count,0]=i
                    ijlk[count,1]=j
                    ijlk[count,2]=l
                    ijlk[count,3]=k
                    Yec[count]=Yn[i*K+l]*Yn[j*K+k]
                    count +=1


        Compactify=csr_matrix((values,(row,cols)),shape=(N*K+E*K*K,N*K+(E*K*K/2)));
        #Yc = concatenate ((Yn,Yec))
        #Yuc_reconstructed=Compactify*Yc;
        #areEqualVectors(Y, Yuc_reconstructed)
        if (LEARN_METHOD == "objassoc"):
            #X with associative and non-associative features filled correctly
            X_small = C*X_sparse
            # Add the example to the list
            examples.append(((X_small, edges, N,example_num ), (Y_s,N,max_target,Compactify,ijlk)))
        else:
            # Add the example to the list
            examples.append(((X_sparse, edges, N,example_num ), (Y_s,N,max_target,Compactify,ijlk)))

    NUM_CLASSES = max_target
    LOSS_WEIGHTS = zeros(K)

    # finding loss weights for training for optmized macro
	# these need to be divided by N(scene specific) to get the weights
    hsum=0.0
    for l in xrange(0,K):
        if(class_counts[l]!=0): # in treaining time, none of this shoule be 0, it nis
            hsum+=(1.0/class_counts[l])
        
    for l in xrange(0,K):
        if(class_counts[l]!=0): # in treaining time, none of this shoule be 0, it nis
            LOSS_WEIGHTS[l]=1.0/(hsum*class_counts[l])
        else:
            LOSS_WEIGHTS[l]=1.0/(K)


        print LOSS_WEIGHTS[l]
        print class_counts[l]
        
    
    # #print out some very useful statistics.
    #print len(examples),'examples read'
    return examples
    

def get_index(edges,u,v):
    for i in xrange(0,edges.shape[0]):
        if (edges[i,0] == u and edges[i,1] == v):
            return i
    assert(2 == 1) # should never reach here

def init_model(sample, sm, sparm):

    """Store the number of features and classes in the model."""
    # Note that these features will be stored in the model and written
    # when it comes time to write the model to a file, and restored in
    # the classifier when reading the model from the file.
    ##print sample[0][0].shape[0]
    global NUM_CLASSES
    #sm.num_features = sample[0][0][0].shape[0]
    sm.num_features = sample[0][0][0].get_shape()[0]
    sm.num_classes = NUM_CLASSES
    print 'num of classes: ', sm.num_classes
    sm.size_psi = sm.num_features
    print 'size_psi set to: ',sm.size_psi

thecount = 0


def lp_training_opt_warm(X,Y,sm,sparm):
    global LP_LIST
    global ITER
    y = Y[0]
    K = sm.num_classes
    w = sm.w
    ijlk = Y[4]
    compactify = Y[3]
    edge = X[1]
    E = edge.shape[0]
    N = X[2]

    if(len(LP_LIST) <= X[3]):
        assert(len(LP_LIST) == X[3])
        lp = glpk.LPX()        # Create empty problem instance
        lp.name = 'training'     # Assign symbolic name to problem
        lp.obj.maximize = True # Set this as a maximization problem
        lp.cols.add(N*K+(E*K*K/2))         # Append three columns to this instance
        for c in lp.cols:      # Iterate over all columns
            if (c.index < N*K) :
                c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
               ##print c.name
            else:
                index = c.index - N*K
                c.name = 'y_%d_%d_%d_%d' %( ijlk[index,0] , ijlk[index,1],ijlk[index,2],ijlk[index,3] )
               ##print c.name
            c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1
        lp.rows.add(3*E*K*K/2 )# + N) # N stands for sum=1 constraints
        for r in lp.rows:      # Iterate over all rows
            r.name = 'p%d' %  r.index # Name them

        for i in xrange(0,E*K*K): # y_i^l>y_ij^lk and y_j^k >= y_ij^lk
            lp.rows[i].bounds = 0, None
        for i in xrange(E*K*K,3*E*K*K/2): #y_i^l + y_j^k \<= 1+y_ij^lk
            lp.rows[i].bounds = None,1
        #for i in xrange(3*E*K*K/2,3*E*K*K/2 + N): #sum=1
         #   lp.rows[i].bounds = 1,1

        t = []
        for n in xrange(0, E * K * K / 2):
            u = ijlk[n, 0]
            v = ijlk[n, 1]
            l = ijlk[n, 2]
            k = ijlk[n, 3]
            a = int(u * K + l) # index of y_i^l
            b = int(v * K + k) # index of y_j^k
            c = N * K + n # index of y_ij^lk
            ec = n
            t.append((ec, a, 1))
            t.append((ec, c, -1))
            ec += E * K * K / 2
            t.append((ec, b, 1))
            t.append((ec, c, -1))
            ec += E * K * K / 2
            t.append((ec, a, 1))
            t.append((ec, b, 1))
            t.append((ec, c, -1))
        lp.matrix = t
        print 'new LP 2 created'
        LP_LIST.append(lp)
    else:
        lp=LP_LIST[X[3]];
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance



    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x*compactify).todense())[0]).tolist()
    for index in xrange(0,N*K):
        if(y[index,0] == 1):
            coeff_list[index] = coeff_list[index]-(1.0/(N*K))
        else:
            coeff_list[index] = coeff_list[index]+(1.0/(N*K))
    lp.obj[:] = coeff_list

    ##print lp.obj[:]


    '''for n in xrange(0, N):
        r = 3*E*K*K/2+n
        for i in xrange(0,K):
            c = n*K+i
            t.append((r,c,1))'''

    ##print len(t)
    #lp.warm_up();
    lp.simplex()#glpk.LPX.MSG_ALL)
    #----------------------------
    # iter changes.. which are not required with warm restart..
    #if(ITER < 4):
    #  numIt = 1000
    #elif(ITER < 8):
    #  numIt = 10000
    #else:
    #  numIt = 100000
    #lp.simplex(it_lim=numIt)
    #--------------------------------
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling.T.shape[0],labeling.T.shape[1]
    y_compact = csr_matrix(labeling.T,dtype='d')
    y_uncompact = compactify*y_compact
    ymax = (y_uncompact,N,K,compactify,ijlk)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in xrange(0,y_uncompact.shape[0]):
        if (y_uncompact[c,0] == 1):
            c1 += 1
        elif(y_uncompact[c,0] ==0):
            c0 += 1
        elif (y_uncompact[c,0] == 0.5):
            ch += 1
        else:
            cr +=1
            assert(round (y_uncompact[c,0],2) == 0.00)
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    #print 'score : ' , round(score,2), ' score2: ',score2;
    #print 'loss: ',loss(Y,ymax,sparm)
    #print '\n'
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    #LP_LIST[X[3]]=lp
    return ymax

def lp_training_sum1_opt(X,Y,sm,sparm):
    y = Y[0]
    K = sm.num_classes
    w = sm.w
    ijlk = Y[4]
    compactify = Y[3]
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'training'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(N*K+(E*K*K/2))         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance

    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            ##print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d_%d_%d_%d' %( ijlk[index,0] , ijlk[index,1],ijlk[index,2],ijlk[index,3] )
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1


    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x*compactify).todense())[0]).tolist()
    for index in xrange(0,N*K):
        if(y[index,0] == 1):
            coeff_list[index] = coeff_list[index]-(1.0/(N*K))
        else:
            coeff_list[index] = coeff_list[index]+(1.0/(N*K))
    lp.obj[:] = coeff_list

    ##print lp.obj[:]

    lp.rows.add(3*E*K*K/2  + N) # N stands for sum=1 constraints
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,E*K*K): # y_i^l>y_ij^lk and y_j^k >= y_ij^lk
        lp.rows[i].bounds = 0, None
    for i in xrange(E*K*K,3*E*K*K/2): #y_i^l + y_j^k \<= 1+y_ij^lk
        lp.rows[i].bounds = None,1
    for i in xrange(3*E*K*K/2,3*E*K*K/2 + N): #sum=1
        lp.rows[i].bounds = 1,1

    t = []
    for n in xrange(0, E * K * K / 2):
        u = ijlk[n, 0]
        v = ijlk[n, 1]
        l = ijlk[n, 2]
        k = ijlk[n, 3]
        a = int(u * K + l) # index of y_i^l
        b = int(v * K + k) # index of y_j^k
        c = N * K + n # index of y_ij^lk
        ec = n
        t.append((ec, a, 1))
        t.append((ec, c, -1))
        ec += E * K * K / 2
        t.append((ec, b, 1))
        t.append((ec, c, -1))
        ec += E * K * K / 2
        t.append((ec, a, 1))
        t.append((ec, b, 1))
        t.append((ec, c, -1))
        
    for n in xrange(0, N):
        r = 3*E*K*K/2+n
        for i in xrange(0,K):
            c = n*K+i
            t.append((r,c,1))
    
    ##print len(t)
    lp.matrix = t
    lp.simplex()
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling.T.shape[0],labeling.T.shape[1]
    y_compact = csr_matrix(labeling.T,dtype='d')
    y_uncompact = compactify*y_compact
    ymax = (y_uncompact,N,K,compactify,ijlk)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in xrange(0,y_uncompact.shape[0]):
        if (y_uncompact[c,0] == 1):
            c1 += 1
        elif(y_uncompact[c,0] ==0):
            c0 += 1
        elif (y_uncompact[c,0] == 0.5):
            ch += 1
        else:
            cr +=1
            #print "value of other value : ", y_uncompact[c,0]
            assert(round (y_uncompact[c,0],2) == 0.00)
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    #print 'score : ' , round(score,2), ' score2: ',score2;
    #print 'loss: ',loss(Y,ymax,sparm)
    #print '\n'
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    return ymax

def lp_training_sum1_opt_IP_warm(X,Y,sm,sparm):
    global LP_LIST
    global ITER
    y = Y[0]
    K = sm.num_classes
    w = sm.w
    ijlk = Y[4]
    compactify = Y[3]
    edge = X[1]
    E = edge.shape[0]
    N = X[2]

    if(len(LP_LIST) <= X[3]):
        assert(len(LP_LIST) == X[3])
        lp = glpk.LPX()        # Create empty problem instance
        lp.name = 'training'     # Assign symbolic name to problem
        lp.obj.maximize = True # Set this as a maximization problem
        lp.cols.add(N*K+(E*K*K/2))         # Append three columns to this instance
        for c in lp.cols:      # Iterate over all columns
            if (c.index < N*K) :
                c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
               ##print c.name
            else:
                index = c.index - N*K
                c.name = 'y_%d_%d_%d_%d' %( ijlk[index,0] , ijlk[index,1],ijlk[index,2],ijlk[index,3] )
               ##print c.name
            c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1
        lp.rows.add(3*E*K*K/2  + N) # N stands for sum=1 constraints
        for r in lp.rows:      # Iterate over all rows
            r.name = 'p%d' %  r.index # Name them

        for i in xrange(0,E*K*K): # y_i^l>y_ij^lk and y_j^k >= y_ij^lk
            lp.rows[i].bounds = 0, None
        for i in xrange(E*K*K,3*E*K*K/2): #y_i^l + y_j^k \<= 1+y_ij^lk
            lp.rows[i].bounds = None,1
        for i in xrange(3*E*K*K/2,3*E*K*K/2 + N): #sum=1
            lp.rows[i].bounds = 1,1

        t = []
        for n in xrange(0, E * K * K / 2):
            u = ijlk[n, 0]
            v = ijlk[n, 1]
            l = ijlk[n, 2]
            k = ijlk[n, 3]
            a = int(u * K + l) # index of y_i^l
            b = int(v * K + k) # index of y_j^k
            c = N * K + n # index of y_ij^lk
            ec = n
            t.append((ec, a, 1))
            t.append((ec, c, -1))
            ec += E * K * K / 2
            t.append((ec, b, 1))
            t.append((ec, c, -1))
            ec += E * K * K / 2
            t.append((ec, a, 1))
            t.append((ec, b, 1))
            t.append((ec, c, -1))

        for n in xrange(0, N):
            r = 3*E*K*K/2+n
            for i in xrange(0,K):
                c = n*K+i
                t.append((r,c,1))

        lp.matrix = t
        print 'new LP 2 created'
        LP_LIST.append(lp)
    else:
        lp=LP_LIST[X[3]];
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance



    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x*compactify).todense())[0]).tolist()
    for index in xrange(0,N*K):
        if(y[index,0] == 1):
            coeff_list[index] = coeff_list[index]-(1.0/(N*K))
        else:
            coeff_list[index] = coeff_list[index]+(1.0/(N*K))
    lp.obj[:] = coeff_list

    ##print lp.obj[:]



    ##print len(t)
    #lp.warm_up();
    retval=lp.simplex();
    assert retval == None
    ones=bitarray()
    zeros=bitarray()

    if lp.status=='opt':
        for c in lp.cols:      # Iterate over all columns
            if (c.index < N*K) :
                if(c.primal==1):
                    ones.append(True);
                else:
                    ones.append(False);
                if(c.primal==0):
                    zeros.append(True);
                else:
                    zeros.append(False);
                c.kind=int
        retval=lp.integer()
        assert retval == None
        if lp.status=='opt':
            for c in lp.cols:      # Iterate over all columns
                if (c.index < N*K) :
                    if(ones[c.index] and not c.primal==1):
                        print '#'
                    if(zeros[c.index] and not c.primal==0):
                        print '@'
        else:
            print '^^^'

    else:
        print '&&&'

    #----------------------------
    # iter changes.. which are not required with warm restart..
    #if(ITER < 4):
    #  numIt = 1000
    #elif(ITER < 8):
    #  numIt = 10000
    #else:
    #  numIt = 100000
    #lp.simplex(it_lim=numIt)
    #--------------------------------
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling.T.shape[0],labeling.T.shape[1]
    y_compact = csr_matrix(labeling.T,dtype='d')
    y_uncompact = compactify*y_compact
    ymax = (y_uncompact,N,K,compactify,ijlk)
    '''c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in xrange(0,y_uncompact.shape[0]):
        if (y_uncompact[c,0] == 1):
            c1 += 1
        elif(y_uncompact[c,0] ==0):
            c0 += 1
        elif (y_uncompact[c,0] == 0.5):
            ch += 1
        else:
            cr +=1
            assert(round (y_uncompact[c,0],2) == 0.00)'''
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    #print 'score : ' , round(score,2), ' score2: ',score2;
    #print 'loss: ',loss(Y,ymax,sparm)
    #print '\n'
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    #LP_LIST[X[3]]=lp
    return ymax

def lp_training_sum1(X,Y,sm,sparm):
    y = Y[0]
    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            ##print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1


    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x).todense())[0]).tolist()
    for index in xrange(0,N*K):
        if(y[index,0] == 1):
            coeff_list[index] = coeff_list[index]-(1.0/(N*K))
        else:
            coeff_list[index] = coeff_list[index]+(1.0/(N*K))
    lp.obj[:] = coeff_list

    ##print lp.obj[:]

    lp.rows.add(3*E*K*K + N)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E*K*K,3*E*K*K + N):
        lp.rows[i].bounds = 1,1

    t = []
    for e in xrange(0,edge.shape[0]):
        u = edge[e,0]
        v = edge[e,1]
        n = -1
        for i in xrange(0,K):
            for j in xrange(0,K):
                n += 1
                a = int(u*K + i)
                b = int(v*K + j)
                c = N*K + e*K*K + i*K + j
                ec = e*K*K + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    for e in xrange(0,N):
        r = 3*E*K*K+e
        for i in xrange(0,K):
            c = e*K+i
            t.append((r,c,1))

    ##print len(t)
    lp.matrix = t
    lp.simplex()
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling.T.shape[0],labeling.T.shape[1]
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    #print 'score : ' , round(score,2), ' score2: ',score2;
    #print 'loss: ',loss(Y,ymax,sparm)
    #print '\n'
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    return ymax

def lp_inference_sum1(X,sm,sparm):
    start = time.clock() 

    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            #c.kind=int
            ##print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    ##print lp.obj[:]

    lp.rows.add(3*E*K*K+N)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E*K*K,3*E*K*K + N):
        lp.rows[i].bounds = 1,1

    t = []
    for e in xrange(0,edge.shape[0]):
        u = edge[e,0]
        v = edge[e,1]
        n = -1
        for i in xrange(0,K):
            for j in xrange(0,K):
                n += 1
                a = int(u*K + i)
                b = int(v*K + j)
                c = N*K + e*K*K + i*K + j
                ec = e*K*K + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    for e in xrange(0,N):
        r = 3*E*K*K+e
        for i in xrange(0,K):
            c = e*K+i
            t.append((r,c,1))


    ##print len(t)
    lp.matrix = t
    lp.simplex()
    lpFin = time.clock()
    print "Time for LP:", (lpFin-start)
    sys.stdout.flush()
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    #print labeling.T
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value,2) ==  round(score,2))
    return ymax

def lp_inference_sum1_IP(X,sm,sparm,LE):
    start = time.clock() 

    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K):
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            c.kind=int
            ##print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    ##print lp.obj[:]

    lp.rows.add(3*E*K*K+N)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E*K*K,3*E*K*K + N):
		if (LE == False) :
			lp.rows[i].bounds = 1,1  ##SUM = 1
		else:
			lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1

    t = []
    for e in xrange(0,edge.shape[0]):
        u = edge[e,0]
        v = edge[e,1]
        n = -1
        for i in xrange(0,K):
            for j in xrange(0,K):
                n += 1
                a = int(u*K + i)
                b = int(v*K + j)
                c = N*K + e*K*K + i*K + j
                ec = e*K*K + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    for e in xrange(0,N):
        r = 3*E*K*K+e
        for i in xrange(0,K):
            c = e*K+i
            t.append((r,c,1))


    ##print len(t)
    lp.matrix = t
    retval=lp.simplex();

    lpFin = time.clock()
    print "Time for LP:", (lpFin-start)

    assert retval == None
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.kind=int


    retval=lp.integer(tm_lim=300000)


    MIPFin = time.clock()
    print "Time for MIP:", (MIPFin-lpFin)

    assert retval == None or retval == "tmlim"
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    if(retval == None):
        labeling = asmatrix(array([c.primal for c in lp.cols]))
    #print labeling.T
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value,2) ==  round(score,2))
    return ymax

def lp_inference_qbpo_sum1_IP(X,sm,sparm):

    start = time.clock()

    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]

 ############ solve QBPO
    qpbo = QPBO(N*K,E*K*K)        # Create empty problem instance
    qpbo.add_node(N*K)

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    #print w_list    y = Y[0]
    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    qpbo = QPBO(N*K,E*K*K)        # Create empty problem instance
    qpbo.add_node(N*K)

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    #print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x).todense())[0]).tolist()

    for index in xrange(0,N*K):
        qpbo.add_term(index,0,-coeff_list[index]);

    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        n1 = int(u*K + l)
        n2 = int(v*K + k)
        qpbo.add_term(n1,n2,0,0,0,-coeff_list[index+N*K])
    ##print lp.obj[:]
    qpbo.solve();
    qpbo.compute_weak_persistencies();

 ############ done


    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K):
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            c.kind=int
            ##print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    ##print lp.obj[:]

    lp.rows.add(3*E*K*K+N)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E*K*K,3*E*K*K + N):
        lp.rows[i].bounds = 1,1

    t = []
    for e in xrange(0,edge.shape[0]):
        u = edge[e,0]
        v = edge[e,1]
        n = -1
        for i in xrange(0,K):
            for j in xrange(0,K):
                n += 1
                a = int(u*K + i)
                b = int(v*K + j)
                c = N*K + e*K*K + i*K + j
                ec = e*K*K + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    for e in xrange(0,N):
        r = 3*E*K*K+e
        for i in xrange(0,K):
            c = e*K+i
            t.append((r,c,1))


    ##print len(t)
    lp.matrix = t


    ########## Initialize basis with QBPO
    for col in lp.cols:
        if(col.index<N*K):
            if(qpbo.get_label(col.index)==1):
                col.status="nu"
            elif(qpbo.get_label(col.index)==0):
                col.status="nl"
            else:
                col.status="bs"
            
    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        l1 = qpbo.get_label(int(u*K + l))
        l2 = qpbo.get_label(int(v*K + k))
        if(l1*l2 == 1): #and=1 , y1y2 stand for and
            lp.cols[N*K+index].status="nu"
        elif(l1*l2==0): # and=0
            lp.cols[N*K+index].status="nl"
        else:
            lp.cols[N*K+index].status="bs"




    ###########
    retval=lp.simplex();

    lpFin = time.clock()
    print "Time for LP:", (lpFin-start)

    assert retval == None

    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.kind=int


    retval=lp.integer()


    MIPFin = time.clock()
    print "Time for MIP:", (MIPFin-lpFin)

    assert retval == None
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    #print labeling.T
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value,2) ==  round(score,2))
    return ymax


def lp_inference_qbpo(X,sm,sparm):

    start = time.clock()

    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
 ############ solve QBPO
    qpbo = QPBO(N*K,E*K*K)        # Create empty problem instance
    qpbo.add_node(N*K)

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    #print w_list    y = Y[0]
    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    qpbo = QPBO(N*K,E*K*K)        # Create empty problem instance
    qpbo.add_node(N*K)

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    #print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x).todense())[0]).tolist()

    for index in xrange(0,N*K):
        qpbo.add_term(index,0,-coeff_list[index]);

    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        n1 = int(u*K + l)
        n2 = int(v*K + k)
        qpbo.add_term(n1,n2,0,0,0,-coeff_list[index+N*K])
    ##print lp.obj[:]
    qpbo.solve();
    qpbo.compute_weak_persistencies();

 ############ done

    labellist = [];
    for n in xrange(0,N*K):
        l = qpbo.get_label(n);
        #print n,l
        if(l == 0):
            labellist.append(0);
        elif(l ==1):
            labellist.append(1);
        else:
            labellist.append(0.5);

    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        l1 = labellist[int(u*K + l)]
        l2 = labellist[int(v*K + k)]
        if(l1*l2 == 0.25):
            if(coeff_list[index+N*K]>0):
                labellist.append(0.5)
            else:
                labellist.append(0)
        else:
            labellist.append(l1*l2);

    labeling = asmatrix(array([labellist]))
    #print labeling.T
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
  
    Fin = time.clock()
    print "Time for Qbpo:", (Fin-start)
    sys.stdout.flush()
    #score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    
    return ymax


def lp_inference_random_w(X,sm,sparm):

    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            ##print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    ##print lp.obj[:]

    lp.rows.add(3*E*K*K+N)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E*K*K,3*E*K*K + N):
        lp.rows[i].bounds = 1,1

    t = []
    for e in xrange(0,edge.shape[0]):
        u = edge[e,0]
        v = edge[e,1]
        n = -1
        for i in xrange(0,K):
            for j in xrange(0,K):
                n += 1
                a = int(u*K + i)
                b = int(v*K + j)
                c = N*K + e*K*K + i*K + j
                ec = e*K*K + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    for e in xrange(0,N):
        r = 3*E*K*K+e
        for i in xrange(0,K):
            c = e*K+i
            t.append((r,c,1))


    ##print len(t)
    lp.matrix = t
    lp.simplex()
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    #print labeling.T
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value,2) ==  round(score,2))
    return ymax

def lp_training(X,Y,sm,sparm):
    y = Y[0]
    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            ##print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1


    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x).todense())[0]).tolist()
    for index in xrange(0,N*K):
        if(y[index,0] == 1):
            coeff_list[index] = coeff_list[index]-(1.0/(N*K))
        else:
            coeff_list[index] = coeff_list[index]+(1.0/(N*K))
    lp.obj[:] = coeff_list

    ##print lp.obj[:]

    lp.rows.add(3*E*K*K)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1

    t = []
    for e in xrange(0,edge.shape[0]):
        u = edge[e,0]
        v = edge[e,1]
        n = -1
        for i in xrange(0,K):
            for j in xrange(0,K):
                n += 1
                a = int(u*K + i)
                b = int(v*K + j)
                c = N*K + e*K*K + i*K + j
                ec = e*K*K + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E*K*K
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))

    ##print len(t)
    lp.matrix = t
    lp.simplex()
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling.T.shape[0],labeling.T.shape[1]
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    print "LP Counts:"
    print 'number of 1s: %d' % c1
    print 'number of 0s: %d' % c0
    print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    print 'score : ' , round(score+loss(Y,ymax,sparm),2) #, ' score2: ',score2;
    #print 'loss: ',loss(Y,ymax,sparm)
    #print '\n'
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    return ymax

def lp_training_qpbo(X,Y,sm,sparm):
    y = Y[0]
    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    qpbo = QPBO(N*K,E*K*K)        # Create empty problem instance
    qpbo.add_node(N*K)
    #print "N:",N," K: ", K
    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    #print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x).todense())[0]).tolist()
    for index_n in xrange(0,N):
        flag = 0
        if(y[index_n*K:(index_n+1)*K-1,0].sum() == 1):
            flag = 1
        for index_k in xrange(0,K):
            if(y[index_n*K+index_k,0] == 1):
                coeff_list[index_n*K+index_k] = coeff_list[index_n*K+index_k]-(1.0/(N*K))
            elif(flag==1):
                coeff_list[index_n*K+index_k] = coeff_list[index_n*K+index_k]+(1.0/(N*K))
    

    for index in xrange(0,N*K):
        qpbo.add_term(index,0,-coeff_list[index]);

    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        n1 = int(u*K + l)
        n2 = int(v*K + k)
        qpbo.add_term(n1,n2,0,0,0,-coeff_list[index+N*K])
    ##print lp.obj[:]
    qpbo.solve();
    qpbo.compute_weak_persistencies();

    labellist = [];
    for n in xrange(0,N*K):
        l = qpbo.get_label(n);
        #print n,l
        if(l == 0):
            labellist.append(0);
        elif(l ==1):
            labellist.append(1);
        else:
            labellist.append(0.5);

    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        l1 = labellist[int(u*K + l)]
        l2 = labellist[int(v*K + k)]
        if(l1*l2 == 0.25):
            if(coeff_list[index+N*K]>0):
                labellist.append(0.5)
            else:
                labellist.append(0)
        else:
            labellist.append(l1*l2);
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([labellist]))
    #print labeling.T.shape[0],labeling.T.shape[1]
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    
    for c in labellist:
        if (c == 1):
            c1 += 1
        elif(c ==0):
            c0 += 1
        else:
            ch +=1
    #print "QPBO counts:"
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    
 #   score = asarray((w_mat*x*ymax[0]).todense())[0][0];

    #print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    #print 'score : ' , round(score+loss(Y,ymax,sparm),2)
    #print 'loss: ',loss(Y,ymax,sparm)
    #print '\n'
    
    #assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    return ymax

def lp_training_qpbo_macro(X,Y,sm,sparm):
    y = Y[0]
    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    qpbo = QPBO(N*K,E*K*K)        # Create empty problem instance
    qpbo.add_node(N*K)

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    #print w_list
    ##print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray((w_mat*x).todense())[0]).tolist()
    for index in xrange(0,N*K):
        classt=index%K
        if(y[index,0] == 1):
            coeff_list[index] = coeff_list[index]-(1.0*LOSS_WEIGHTS[classt]/N)
        else:
            coeff_list[index] = coeff_list[index]+(1.0*LOSS_WEIGHTS[classt]/N)


    for index in xrange(0,N*K):
        qpbo.add_term(index,0,-coeff_list[index]);

    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        n1 = int(u*K + l)
        n2 = int(v*K + k)
        qpbo.add_term(n1,n2,0,0,0,-coeff_list[index+N*K])
    ##print lp.obj[:]
    qpbo.solve();
    qpbo.compute_weak_persistencies();

    labellist = [];
    for n in xrange(0,N*K):
        l = qpbo.get_label(n);
        #print n,l
        if(l == 0):
            labellist.append(0);
        elif(l ==1):
            labellist.append(1);
        else:
            labellist.append(0.5);

    for index in xrange(0,E*K*K):
        u = edge[int(index/(K*K)),0]
        v = edge[int(index/(K*K)),1]
        l = int((index%(K*K))/K)
        k = int((index%(K*K))%K)

        l1 = labellist[int(u*K + l)]
        l2 = labellist[int(v*K + k)]
        if(l1*l2 == 0.25):
            if(coeff_list[index+N*K]>0):
                labellist.append(0.5)
            else:
                labellist.append(0)
        else:
            labellist.append(l1*l2);
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([labellist]))
    #print labeling.T.shape[0],labeling.T.shape[1]
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0

    for c in labellist:
        if (c == 1):
            c1 += 1
        elif(c ==0):
            c0 += 1
        else:
            ch +=1
    #print "QPBO counts:"
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch

#    score = asarray((w_mat*x*ymax[0]).todense())[0][0];

    #print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    #print 'score : ' , round(score+loss(Y,ymax,sparm),2)
    #print 'loss: ',loss(Y,ymax,sparm)
    #print '\n'

    #assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    return ymax

def lp_inference(X,sm,sparm):
    start = time.clock() 
    K = sm.num_classes
    w = sm.w
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    print "num cols: ",X[0].shape[1] , " K: " , K , " N: ",N , " E: ",E
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    #print X[0].shape[1]
    #print N,E,K
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            ##print c.name
        else:
            index = c.index - N*K
            #print index
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            ##print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    ##print lp.obj[:]

    lp.rows.add(3*E*K*K)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1

    t = []
    for e in xrange(0,edge.shape[0]):
        u = edge[e,0]
        v = edge[e,1]
        n = -1
        for i in xrange(0,K):
            for j in xrange(0,K):
                n += 1
                a = int(u*K + i)
                b = int(v*K + j)
                c = N*K + e*K*K + i*K + j
                ec = e*K*K + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
   #             print ec,a,c
                ec += E*K*K
                t.append((ec,b,1))
                t.append((ec,c,-1))
   #             print ec,c,b
                ec += E*K*K
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    #            print ec,a,b,c

    ##print len(t)
    lp.matrix = t
    lp.simplex() #it_lim=10000)
    elapsed = (time.clock() - start) 
    print "Time for LP:", elapsed
#    lp.simplex()
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling.T
    ymax = (csr_matrix(labeling.T,dtype='d'),N,K)
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value,2) ==  round(score,2))
    return ymax

def classification_score(x,y,sm,sparm):
    """Return an example, label pair discriminant score."""
    # Utilize the svmapi.Model convenience method 'classify'.
    score = sm.svm_model.classify(psi(x,y,sm,sparm))
    global thecount
    thecount += 1
    if (sum(abs(w) for w in sm.w)):
        import pdb; pdb.set_trace()
    return score

def classify_example(x, sm, sparm):
    """Returns the classification of an example 'x'."""
    global CLASSIFY_METHOD
    #y = (mat(ones((1,x[0].shape[1]))),x[2],sm.num_classes)
    #l = lp_inference(x,sm,sparm)
    
    if(CLASSIFY_METHOD == "sum1.IP"):
        l = lp_inference_sum1_IP(x,sm,sparm,False)
    elif(CLASSIFY_METHOD == "sumLE1.IP"):
        l = lp_inference_sum1_IP(x,sm,sparm,True)
    elif(CLASSIFY_METHOD == "sum1"):
        l = lp_inference_sum1(x,sm,sparm)
    elif(CLASSIFY_METHOD == "qbpo.sum1.IP"):
        l = lp_inference_qbpo_sum1_IP(x,sm,sparm)
    elif(CLASSIFY_METHOD == "qbpo"):
        l = lp_inference_qbpo(x,sm,sparm)
   
    return l

def areEqualVectors(V1,V2):
    for i in xrange(0,V1.shape[0]):
        assert(round(V1[i,0]*2, 0)==round(V2[i,0]*2, 0))
        
def find_most_violated_constraint(x, y, sm, sparm):
    """Returns the most violated constraint for example (x,y)."""
    global LOSS_METHOD
    # Similar, but include the loss.
    #print
    #print "MOST VIOLATED Constraint"
    # l1 = lp_training(x,y,sm,sparm)
    if(LOSS_METHOD== "micro"):
        l2 = lp_training_qpbo(x,y,sm,sparm)
    else:
        l2 = lp_training_qpbo_macro(x,y,sm,sparm)
    #print "l1:"
    #for i in xrange(l1[1]*sm.num_classes):
        #print l1[0][i,0],l2[0][i,0]
     #   assert l1[0][i,0] == l2[0][i,0]
    #print "l2"
    #print l2[0]

    #print
    #assert (l1[0] == l2[0])
    #l = lp_training_opt(x,y,sm,sparm)
    #l = lp_training(x,y,sm,sparm)

    ##print l.T
    return l2

def psi(x, y, sm, sparm):
    
    """Returns the combined feature vector Psi(x,y)."""
    # Return the product of x and y
    ##print x[0].shape[0]
    ##print x[0].shape[1]
    ##print y.shape[0]
    ##print y.shape[1]
    return svmapi.Sparse(((x[0]*y[0]).todense()))
    

def loss(Y, Ybar, sparm):
    global LOSS_METHOD
    if(LOSS_METHOD == "macro"):
        return loss_macro(Y, Ybar, sparm);
    else:
        return loss_micro(Y, Ybar, sparm);

def loss_micro(Y, Ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    N = Y[1]
    K = Y[2]
    y= Y[0]
    
    #print "N:",N," K: ", K #,y.shape[0],y.shape[1]
    ybar = Ybar[0] 
    yDiff=y- ybar;
    sum=0.0;
    size=N*K

    for index_n in xrange(0,N):
        for index_k in xrange(0,K):
            if (y[index_n*K : index_n*K+K-1,0].sum() == 1):
                if yDiff[index_n*K+index_k,0]>0:
                    sum+=yDiff[index_n*K+index_k,0]
                else:
                    sum-=yDiff[index_n*K+index_k,0]
            
    return sum/size;

def loss_macro(Y, Ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    N = Y[1]
    K = Y[2]
    y= Y[0]

   # #print N,K,y.shape[0],y.shape[1]
    ybar = Ybar[0]
    yDiff=y- ybar;
    sum=0.0;
    size=N*K

    for n in xrange(0,N):
        for k in xrange(0,K):
            index=n*K+k
            if yDiff[index,0]>0:
                sum+=LOSS_WEIGHTS[k]*yDiff[index,0]/N
            else:
                sum-=LOSS_WEIGHTS[k]*yDiff[index,0]/N # adding absolute diff

    return sum;

def write_label(fileptr, y):
    K= y[2]
    N = y[1]
    for node in xrange(0,N):
        for label in xrange(0,K):
            if(y[0][node*K+label,0] != 0):
                s = repr(node+1)+':'+repr(label+1)+':'+repr(y[0][node*K+label,0])
                print>>fileptr,s,
    print>>fileptr

def print_iteration_stats(ceps, cached_constraint, sample, sm,
                          cset, alpha, sparm):
    """Called just before the end of each cutting plane iteration.

    This is called just before the end of each cutting plane
    iteration, primarily to #print statistics.  The 'ceps' argument is
    how much the most violated constraint was violated by.  The
    'cached_constraint' argument is true if this constraint was
    constructed from the cache.
    
    The default behavior is that nothing is #printed."""
    # for every 10th iteration save the model file 
    global ITER
    ITER += 1;
    if(ITER%1 == 0):
        filename = "imodels/model.c"+ `sparm.c`  + ".m" + `ITER`;
        print "writing intermediate model: ", filename
        write_model(filename, sm, sparm)
    # #printig the weight vector
    #w_list = [sm.w[i] for i in xrange(0,sm.size_psi)]
    ##print w_list

def write_model(filename, sm, sparm):
    import cPickle, bz2
    cPickle.dump(sm,file(filename,'w'))

def read_model(filename, sparm):
    import cPickle, bz2
    return cPickle.load(file(filename))



def evaluation_class_pr(Y,Ybar,K,N,sparm):
    y = Y[0]   
    ybar = Ybar[0]
    truecount = zeros((K,1))
    predcount = zeros((K,1))
    tpcount = zeros((K,1))
    confusionMatrix=zeros((K,K))
    singlepredcount = zeros((K,1))
    confusionMatrixWMultiple=zeros((K,K))
    multipleClasses=zeros((K,1))
    zeroClasses=zeros((K,1))
    prec = zeros((K,1))
    recall = zeros((K,1))
    for node in xrange(0,N):
        numPositives=0;
        predClass=-1;
        actualClass=-1;
        for label in xrange(0,K):
            if(y[node*K+label,0] == 1):
                truecount[label,0] += 1;
                actualClass=label
        for label in xrange(0,K):
            if(ybar[node*K+label,0] == 1):
                predcount[label,0] += 1;
                numPositives+=1;
                predClass=label;
                confusionMatrixWMultiple[label,actualClass]+=1;
            if((y[node*K+label,0] == 1) and (ybar[node*K+label,0] == 1)):
                tpcount[label,0] += 1

        if(numPositives==0):
            zeroClasses[actualClass,0]+=1
        elif(numPositives>1):
            multipleClasses[actualClass,0]+=1
        else:
            confusionMatrix[predClass,actualClass]+=1
            singlepredcount[predClass,0] += 1;
    for label in xrange(0,K):
        if(predcount[label,0] != 0):
            prec[label,0] = tpcount[label,0]/float(predcount[label,0])
        if(truecount[label,0] !=0):
            recall[label,0] = tpcount[label,0]/float(truecount[label,0])
    return (tpcount,truecount,predcount,confusionMatrix,zeroClasses,multipleClasses,confusionMatrixWMultiple,singlepredcount)

def evaluation_class_pr_sum1(Y,Ybar,K,N,sparm):
    y = Y[0]
    ybar = Ybar[0]
    truecount = zeros((K,1))
    predcount = zeros((K,1))
    singlepredcount = zeros((K,1))
    tpcount = zeros((K,1))
    confusionMatrix=zeros((K,K))
    confusionMatrixWMultiple=zeros((K,K))
    multipleClasses=zeros((K,1))
    zeroClasses=zeros((K,1))
    prec = zeros((K,1))
    recall = zeros((K,1))
    f = open('pred.txt','a')
 
    for node in xrange(0,N):
        flag = 0;
        numPositives=0;
        predClass=-1;
        actualClass=-1;
        maxYBar=-1;
        countMax=0
        for label in xrange(0,K):
            if(y[node*K+label,0] == 1):
                truecount[label,0] += 1;
                actualClass=label
            if(maxYBar<ybar[node*K+label,0]):
                maxYBar=ybar[node*K+label,0]
                countMax=0
            if(maxYBar==ybar[node*K+label,0]):
                countMax+=1
        if(actualClass == -1):
            continue;
        maxLabelList=[];
        for label in xrange(0,K):
            if(ybar[node*K+label,0] == maxYBar and maxYBar>0): #suboptimal way, but who cares!
                maxLabelList.append(label);
                predcount[label,0] += 1;
                numPositives+=1;
                predClass=label;
                confusionMatrixWMultiple[label,actualClass]+=1.0/countMax;

        if(numPositives==0):
            zeroClasses[actualClass,0]+=1
        elif(numPositives>=1):
#            multipleClasses[actualClass,0]+=1
#        else:
            if(numPositives>1):
                prediction=maxLabelList[random.randint(0,countMax)]
            else:
                prediction=maxLabelList[0]
            confusionMatrix[prediction,actualClass]+=1
            singlepredcount[prediction,0] += 1;
            if(actualClass==prediction):
                tpcount[prediction,0] += 1
                flag = 1;
				
        print>>f, node+1,flag         
    print>>f, "\n"
    for label in xrange(0,K):
        if(singlepredcount[label,0] != 0):
            prec[label,0] = tpcount[label,0]/float(singlepredcount[label,0])
        if(truecount[label,0] !=0):
            recall[label,0] = tpcount[label,0]/float(truecount[label,0])
    return (tpcount,truecount,predcount,confusionMatrix,zeroClasses,multipleClasses,confusionMatrixWMultiple,singlepredcount)

def evaluation_prec_recall(Y, Ybar, K, N ,sparm):
    y = Y[0]
    ybar = Ybar[0]
    prec = 0.0
    recall = 0.0
    for node in xrange(0,N):
        tp_fn = 0.0
        tp_fp = 0.0
        tp= 0.0 #multiply(y[node*K:node*K+K],ybar[node*K:node*K+K])
        for label in xrange(0,K):
            tp_fn += y[node*K+label,0]*y[node*K+label,0]
            tp_fp += ybar[node*K+label,0]*ybar[node*K+label,0]
            tp += y[node*K+label,0]*ybar[node*K+label,0]
        
        if( tp_fp > 0):
            prec += tp/tp_fp; 
        else:
            prec += 0;
        if( tp_fn > 0):
            recall += tp/tp_fn; 
        else:
            recall += 0;

    ##print "similarity is", sim/N
    return (prec/N, recall/N);


def evaluation_loss(Y, Ybar, K, N ,sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    y = Y[0]
    ybar = Ybar[0]
    sim = 0.0
    for node in xrange(0,N):
        moda = 0.0
        modb = 0.0
        num= 0.0 #multiply(y[node*K:node*K+K],ybar[node*K:node*K+K])
        for label in xrange(0,K):
            moda += y[node*K+label,0]*y[node*K+label,0]
            modb += ybar[node*K+label,0]*ybar[node*K+label,0]
            num += y[node*K+label,0]*ybar[node*K+label,0]
        denom = sqrt(moda)*sqrt(modb)
        if(denom > 0):
            sim += num/(sqrt(moda)*sqrt(modb))
        else:
            sim += 0;
    
    ##print "similarity is", sim/N
    return 1-(sim/N);


def eval_prediction(exnum, (x, y), ypred, sm, sparm, teststats):
    """Accumulate statistics about a single training example.

    Allows accumulated statistics regarding how well the predicted
    label ypred for pattern x matches the true label y.  The first
    time this function is called teststats is None.  This function's
    return value will be passed along to the next call to
    eval_prediction.  After all test predictions are made, the last
    value returned will be passed along to #print_testing_stats.

    On the first call, that is, when exnum==0, teststats==None.  The
    default behavior is that the function does nothing."""
    global ATTRIBUTE_METHOD
    if exnum==0: teststats = []
    #print 'on example',exnum,'predicted',ypred[0].T,'where correct is',y[0].T
    #print 'loss is',evaluation_loss(y, ypred, sm.num_classes , x[2], sparm)
    #teststats.append(evaluation_loss(y, ypred, sm.num_classes , x[2], sparm))
    if(ATTRIBUTE_METHOD == "true"):
        teststats.append(evaluation_class_pr(y, ypred, sm.num_classes , x[2], sparm))
    else:
        teststats.append(evaluation_class_pr_sum1(y, ypred, sm.num_classes , x[2], sparm))
    return teststats


def print_testing_stats_objects(sample, sm, sparm, teststats):

    avgp = zeros((sm.num_classes,1))
    avgr = zeros((sm.num_classes,1))
    tpcount = zeros((sm.num_classes,1))
    truecount = zeros((sm.num_classes,1))
    predcount = zeros((sm.num_classes,1))
    singlepredcount = zeros((sm.num_classes,1))
    aggConfusionMatrix=zeros((sm.num_classes,sm.num_classes),dtype='i')
    aggConfusionMatrixWMultiple=zeros((sm.num_classes,sm.num_classes),dtype='i')
    aggZeroPreds=zeros((sm.num_classes,1))
    aggMultiplePreds=zeros((sm.num_classes,1))
    for t in teststats:
        tpcount += t[0]
        truecount += t[1]
        predcount += t[2]
        singlepredcount += t[7]
        aggConfusionMatrix+=t[3];
        aggZeroPreds +=t[4];
        aggMultiplePreds +=t[5];
        aggConfusionMatrixWMultiple+=t[6];


    total_tc  = 0
    total_pc = 0
    total_tp = 0
    for label in xrange(0,sm.num_classes):
        if(singlepredcount[label,0] != 0):
            avgp[label,0] = tpcount[label,0]/float(singlepredcount[label,0])
        if(truecount[label,0] !=0):
            avgr[label,0] = tpcount[label,0]/float(truecount[label,0])
      #  avgp[label,0] = avgp[label,0]/len(teststats)
      #  avgr[label,0] = avgr[label,0]/len(teststats)
        print "label ",label+1, " prec: " , avgp[label,0], " recall: " ,avgr[label,0], " tp: ", tpcount[label,0], " tc: ", truecount[label,0], " pc: ", singlepredcount[label,0]
        total_tc +=  truecount[label,0]
        total_pc += singlepredcount[label,0]
        total_tp += tpcount[label,0]
    print "prec: ", total_tp/total_pc , "recall: ",total_tp/total_tc ,"tp: ", total_tp, " pc: ", total_pc, "tc: ", total_tc
    #print "Error per Test example: ", teststats
    print "confusion matrix:"
    print aggConfusionMatrix;
    savetxt('conf.txt',aggConfusionMatrix,fmt='%d');

    print "confusion matrix with multiple semantics:"
    print aggConfusionMatrixWMultiple;
    savetxt('confm.txt',aggConfusionMatrixWMultiple,fmt='%d');

    print "num Zeros:"
    print aggZeroPreds;

    print "num Multiples:"
    print aggMultiplePreds;

def print_testing_stats_attributes(sample, sm, sparm, teststats):

    avgp = zeros((sm.num_classes,1))
    avgr = zeros((sm.num_classes,1))
    tpcount = zeros((sm.num_classes,1))
    truecount = zeros((sm.num_classes,1))
    predcount = zeros((sm.num_classes,1))
    singlepredcount = zeros((sm.num_classes,1))
    aggConfusionMatrix=zeros((sm.num_classes,sm.num_classes),dtype='i')
    aggConfusionMatrixWMultiple=zeros((sm.num_classes,sm.num_classes),dtype='i')
    aggZeroPreds=zeros((sm.num_classes,1))
    aggMultiplePreds=zeros((sm.num_classes,1))
    for t in teststats:
        tpcount += t[0]
        truecount += t[1]
        predcount += t[2]
        singlepredcount += t[7]
        aggConfusionMatrix+=t[3];
        aggZeroPreds +=t[4];
        aggMultiplePreds +=t[5];
        aggConfusionMatrixWMultiple+=t[6];


    total_tc  = 0
    total_pc = 0
    total_tp = 0
    for label in xrange(0,sm.num_classes):
        if(predcount[label,0] != 0):
            avgp[label,0] = tpcount[label,0]/float(predcount[label,0])
        if(truecount[label,0] !=0):
            avgr[label,0] = tpcount[label,0]/float(truecount[label,0])
      #  avgp[label,0] = avgp[label,0]/len(teststats)
      #  avgr[label,0] = avgr[label,0]/len(teststats)
        print "label ",label+1, " prec: " , avgp[label,0], " recall: " ,avgr[label,0], " tp: ", tpcount[label,0], " tc: ", truecount[label,0], " pc: ", predcount[label,0]
        total_tc +=  truecount[label,0]
        total_pc += predcount[label,0]
        total_tp += tpcount[label,0]
    print "prec: ", total_tp/total_pc , "recall: ",total_tp/total_tc ,"tp: ", total_tp, " pc: ", total_pc, "tc: ", total_tc
    #print "Error per Test example: ", teststats
    print "confusion matrix:"
    print aggConfusionMatrix;
    savetxt('conf.txt',aggConfusionMatrix,fmt='%d');

    print "confusion matrix with multiple semantics:"
    print aggConfusionMatrixWMultiple;
    savetxt('confm.txt',aggConfusionMatrixWMultiple,fmt='%d');

    print "num Zeros:"
    print aggZeroPreds;

    print "num Multiples:"
    print aggMultiplePreds;

def print_testing_stats(sample, sm, sparm, teststats):
    """#print statistics once classification has finished.

    This is called after all test predictions are made to allow the
    display of any summary statistics that have been accumulated in
    the teststats object through use of the eval_prediction function.

    The default behavior is that nothing is #printed."""
    global ATTRIBUTE_METHOD
    if(ATTRIBUTE_METHOD=="true"):
        print_testing_stats_attributes(sample, sm, sparm, teststats)
    else:
        print_testing_stats_objects(sample, sm, sparm, teststats)

