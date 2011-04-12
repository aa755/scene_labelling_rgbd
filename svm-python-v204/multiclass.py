"""A module for SVM^python for multiclass learning."""

# Thomas Finley, tfinley@gmail.com

from pipes import clone
from turtle import Shape
import svmapi, array
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy.sparse import lil_matrix
import glpk

global NUM_CLASSES
NUM_CLASSES = 0

def read_examples(filename,sparm):
    global NUM_CLASSES
    # Helper function for reading from files.
    def line_reader(lines):
        # returns only non-empty lines
        # strips comments (anything after '#')
        for l in lines:
            i = l.find('#')
            if i == -1: yield l.strip()

    """Parses an input file into an example sequence."""
    # This reads example files of the type read by SVM^multiclass.
    examples = []
    max_target=0
    num_node_feats=0
    num_edge_feats=0

    # Open the file and read each example.
    for input_file in file(filename):
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N = int(input[0][0].strip());
        E = int(input[0][1].strip());



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

    print max_target
    print num_node_feats
    print num_edge_feats

    for input_file in file(filename):
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N = int(input[0][0].strip());
        E = int(input[0][1].strip());

        Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
        Yn= mat(zeros((max_target*N,1)))
        #XY_test = mat(zeros((max_target*num_node_feats + max_target*max_target*num_edge_feats,1)))
        node_map = {}
        edges = mat(zeros((E,2)))
        for i in xrange(0,N):
            target = int(input[i+1][0]);
            Yn[i*max_target+(target-1),0]=1
            # get the segment number
            node_map[int(input[i+1][1])] = i

            # Get the features.
            tokens = [line.split(':') for line in input[i+1][2:]]
            #print [float(v) for k,v in tokens]
            # assumes all features are present
            features = mat([float(v) for k,v in tokens])
            print features
            f = features.transpose()
            #XY_test[(target-1)*num_node_feats:target*num_node_feats,0] += f
            # fill in the Xn matrix
            for j in xrange(0,max_target):
                #print X[j*9:(j+1)*9,j];
                Xn[j*num_node_feats:(j+1)*num_node_feats,i*max_target+(j)] = f.copy();
            #print X
        Xe = mat(zeros((max_target*max_target*num_edge_feats,max_target*max_target*E)))
        Ye = mat(zeros((max_target*max_target*E,1)))
        for i in xrange(N,N+E):
            target1 = int(input[i+1][0]);
            target2 = int(input[i+1][1]);
            Ye[(i-N)*max_target*max_target + (target1-1)*max_target+(target2-1)]=1
            # get the segment numbers
            edges[i-N,0]= node_map[int(input[i+1][2])]
            edges[i-N,1]= node_map[int(input[i+1][3])]

            tokens = [line.split(':') for line in input[i+1][4:]]
            features = mat([float(v) for k,v in tokens])
            print features
            f = features.transpose()
            z = max_target*num_node_feats + (target1-1)*max_target*num_edge_feats+(target2-1)*num_edge_feats
            #XY_test[z : z+num_edge_feats ,0] += f
            # fill in the Xn matrix
            for j in xrange(0,max_target*max_target):
                #print X[j*9:(j+1)*9,j];
                Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();

        print Xe.shape[0]
        print Xe.shape[1]
        a = concatenate ((Xn, mat(zeros((Xn.shape[0],Xe.shape[1])))),1)
        b = concatenate ((mat(zeros((Xe.shape[0],Xn.shape[1]))),Xe),1)
        X = concatenate ((a,b))
        Y = concatenate ((Yn,Ye))
        X_s = lil_matrix(X)
        Y_s = lil_matrix(Y)

        # Add the example to the list
        examples.append(((X_s, edges, N), Y_s))
    NUM_CLASSES = max_target
    # Print out some very useful statistics.
    print len(examples),'examples read'
    return examples
    



def init_model(sample, sm, sparm):

    """Store the number of features and classes in the model."""
    # Note that these features will be stored in the model and written
    # when it comes time to write the model to a file, and restored in
    # the classifier when reading the model from the file.
    #print sample[0][0].shape[0]
    global NUM_CLASSES
    sm.num_features = sample[0][0][0].shape[0]
    sm.num_classes = NUM_CLASSES
    print 'num of classes', sm.num_classes
    sm.size_psi = sm.num_features
    print 'size_psi set to',sm.size_psi

thecount = 0

def lp(X,K,w):
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            #print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            #print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = asmatrix(array(w_list))
    print w_list
    #print (asarray(w*x)[0]).tolist()
    lp.obj[:] = (asarray(w_mat*x)[0]).tolist()
    #print lp.obj[:]

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

    print len(t)
    lp.matrix = t
    lp.simplex()
    print 'Z = %g;' % lp.obj.value,  # Retrieve and print obj func value
    print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # Print struct variable names and primal val
    labeling = [c.primal for c in lp.cols]
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
    print c1
    print c0
    print ch
    print cr
    return labeling

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
    l = lp(x,sm.num_classes,sm.w)
    return l

def find_most_violated_constraint(x, y, sm, sparm):
    """Returns the most violated constraint for example (x,y)."""
    # Similar, but include the loss.
    l = lp(x,sm.num_classes,sm.w)
    return l

def psi(x, y, sm, sparm):
    """Returns the combined feature vector Psi(x,y)."""
    # Return the product of x and y
    
    return svmapi.Sparse((x[0]*y))

def loss(y, ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    assert 1==2
    return 100.0*int(y != ybar)
