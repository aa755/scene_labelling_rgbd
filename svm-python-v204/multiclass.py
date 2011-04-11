"""A module for SVM^python for multiclass learning."""

# Thomas Finley, tfinley@gmail.com

from turtle import Shape
import svmapi, array
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy.sparse import lil_matrix


def read_examples(filename,sparm):
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
    # Open the file and read each example.
    for input_file in file(filename):
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N = int(input[0][0].strip());
        E = int(input[0][1].strip());

        max_target=0
        num_node_feats=0
        num_edge_feats=0

        # find the max class and number of node features -- will work for sparse representation
        for i in xrange(0,N):
            target = int(input[i+1][0]);
            if (max_target<int(target)):
                max_target=int(target)
            tokens = [line.split(':') for line in input[i+1][1:]]
            for k,v in tokens:
                if(num_node_feats<int(k)):
                    num_node_feats=int(k)
        for i in xrange(N,N+E):
            tokens = [line.split(':') for line in input[i+1][2:]]
            for k,v in tokens:
                if(num_edge_feats<int(k)):
                    num_edge_feats=int(k)

        print max_target
        print num_node_feats
        print num_edge_feats

        Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
        Yn= mat(zeros((max_target*N,1)))
        

        for i in xrange(0,N):
            target = int(input[i+1][0]);
            Yn[i*max_target+(target-1),0]=1
            # Get the features.
            tokens = [line.split(':') for line in input[i+1][1:]]
            #print [float(v) for k,v in tokens]
            # assumes all features are present
            features = mat([float(v) for k,v in tokens])
            f = features.transpose()

            # fill in the Xn matrix
            for j in xrange(0,max_target):
                #print X[j*9:(j+1)*9,j];
                Xn[j*num_node_feats:(j+1)*num_node_feats,i*max_target+(j)] = f.copy();

        Xe = mat(zeros((max_target*max_target*num_edge_feats,max_target*max_target*E)))
        Ye = mat(zeros((max_target*max_target*E,1)))
        for i in xrange(N,N+E):
            target1 = int(input[i+1][0]);
            target2 = int(input[i+1][1]);
            Ye[(i-N)*max_target*max_target + (target1-1)*max_target+(target2-1)]=1
            tokens = [line.split(':') for line in input[i+1][2:]]
            features = mat([float(v) for k,v in tokens])
            f = features.transpose()
            z = max_target*num_node_feats + (target1-1)*max_target*num_edge_feats+(target2-1)*num_edge_feats

            # fill in the Xn matrix
            for j in xrange(0,max_target*max_target):
                #print X[j*9:(j+1)*9,j];
                Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();

        #print Xe.shape[0]
        #print Xe.shape[1]
        a = concatenate ((Xn, mat(zeros((Xn.shape[0],Xe.shape[1])))),1)
        b = concatenate ((mat(zeros((Xe.shape[0],Xn.shape[1]))),Xe),1)
        X = concatenate ((a,b))
        Y = concatenate ((Yn,Ye))
        X_s = lil_matrix(X)
        Y_s = lil_matrix(Y)


        # Add the example to the list
        examples.append((X_s, Y_s))

    # Print out some very useful statistics.
    print len(examples),'examples read'
    return examples


def init_model(sample, sm, sparm):
    """Store the number of features and classes in the model."""
    # Note that these features will be stored in the model and written
    # when it comes time to write the model to a file, and restored in
    # the classifier when reading the model from the file.
    #print sample[0][0].shape[0]
    sm.num_features = sample[0][0].shape[0]
    #sm.num_classes = ?
    sm.size_psi = sm.num_features
    print 'size_psi set to',sm.size_psi

thecount = 0
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
    # Construct the discriminant-label pairs.
    scores = [(classification_score(x,c,sm,sparm), c)
              for c in xrange(1,sm.num_classes+1)]
    # Return the label with the max discriminant value.
    return max(scores)[1]

def find_most_violated_constraint(x, y, sm, sparm):
    """Returns the most violated constraint for example (x,y)."""
    # Similar, but include the loss.
    scores = [(classification_score(x,c,sm,sparm)+loss(y,c,sparm), c)
              for c in xrange(1,sm.num_classes+1)]
    ybar = max(scores)[1]
    #print y, ybar
    return max(scores)[1]

def psi(x, y, sm, sparm):
    """Returns the combined feature vector Psi(x,y)."""
    # Return the product of x and y
    
    return svmapi.Sparse((x*y).todense())

def loss(y, ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    return 100.0*int(y != ybar)
