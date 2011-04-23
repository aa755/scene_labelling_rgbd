"""A module for SVM^python for multiclass learning."""

# Thomas Finley, tfinley@gmail.com

import svmapi, array
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy.sparse import lil_matrix
from numpy.ma.core import zeros
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

    print 'number of classes: ', max_target
    print 'number of node features: ', num_node_feats
    print 'number of edge features: ',num_edge_feats

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

        #print Xe.shape[0]
        #print Xe.shape[1]
        a = concatenate ((Xn, mat(zeros((Xn.shape[0],Xe.shape[1])))),1)
        b = concatenate ((mat(zeros((Xe.shape[0],Xn.shape[1]))),Xe),1)
        X = concatenate ((a,b))
        Y = concatenate ((Yn,Ye))
        X_s = X#lil_matrix(X)
        Y_s = Y#lil_matrix(Y)

        # Add the example to the list
        examples.append(((X_s, edges, N), (Y_s,N,max_target)))
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
    print 'num of classes: ', sm.num_classes
    sm.size_psi = sm.num_features
    print 'size_psi set to: ',sm.size_psi

thecount = 0


def lp_training_old(X,Y,sm,sparm):
    K = sm.num_classes
    w = sm.w
    y = Y[0]
    edge = X[1]
    E = edge.shape[0]
    N = X[2]
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1]+N*K)         # Append three columns to this instance
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            #print c.name
        elif (c.index < X[0].shape[1]):
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            #print c.name
        else:
            index = c.index -  X[0].shape[1]
            c.name = 'e_%d_%d' % ( index/K , (index%K)+1)

        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = (X[0])#.todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = asmatrix(array(w_list))
    #print w_list
    #print (asarray(w*x)[0]).tolist()
    loss_coeff = [float(1/float(N*K)) for i in xrange(0,N*K)]
    coeff = (asarray(w_mat*x)[0]).tolist()
    coeff.extend(loss_coeff)
    lp.obj[:] = coeff
    #print lp.obj[:]

    lp.rows.add(3*E*K*K+2*N*K)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E*K*K):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E*K*K,3*E*K*K):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E*K*K,3*E*K*K+N*K):
        lp.rows[i].bounds = None,y[i-3*E*K*K]
    for i in xrange(3*E*K*K+N*K,3*E*K*K+2*N*K):
        lp.rows[i].bounds = y[i-3*E*K*K+N*K], None
 

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
    for n in xrange(0,N*K):
        ec= 3*E*K*K+n
        a= n 
        b =  n+ N*K + E*K*K
        t.append((ec,a,1))
        t.append((ec,b,-1))

    for n in xrange(0,N*K):
       ec = 3*E*K*K+N*K+n
       a = n
       b = n+ N*K + E*K*K
       t.append((ec,a,1))
       t.append((ec,b,1))



    #print len(t)
    lp.matrix = t
    lp.simplex()
    # print 'Z = %g;' % lp.obj.value,  # Retrieve and print obj func value
    # print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                      # Print struct variable names and primal val
    print "objective value = ", lp.obj.value 
    l = []
    for c in lp.cols:
      if(c.index< N*K+E*K*K):
         l.append(c.primal)
    labeling = asmatrix(array(l))
    ymax = (labeling.T,N,K )   
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
      if (c.index<N*K+E*K*K):
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    print 'number of 1s: %d' % c1
    print 'number of 0s: %d' % c0
    print 'number of 0.5s: %d' % ch
    print 'number of 0s: %d' % cr
    score = asarray(w_mat*x*ymax[0])[0][0];
    score2 = sm.svm_model.classify(psi(x,ymax,sm,sparm))
    print '\n score : ' , score, ' score2: ',score2;
    print 'loss: ',loss(Y,ymax,sparm)
    if(lp.obj.value  > 1.1):
      assert (lp.obj.value ==  score +  loss(Y,ymax,sparm)) #(sm.svm_model.classify(psi(x,labeling.T,sm,sparm)) + loss(y,labeling.T,sparm)) )
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
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N*K) :
            c.name = 'y_%d_%d' % ( c.index/K , (c.index%K)+1) # Name them x0, x1, and x2
            #print c.name
        else:
            index = c.index - N*K
            c.name = 'y_%d-%d_%d-%d' % ( edge[int(index/(K*K)),0] ,edge[int(index/(K*K)),1] , int((index%(K*K))/K)+1 , int((index%(K*K))%K)+1)
            #print c.name
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

    x = (X[0])#.todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = asmatrix(array(w_list))
    #print w_list
    #print (asarray(w*x)[0]).tolist()
    coeff_list = (asarray(w_mat*x)[0]).tolist()
    for index in xrange(0,N*K):
        if(y[index,0] == 1):
            coeff_list[index] = coeff_list[index]-(1.0/(N*K))
        else:
            coeff_list[index] = coeff_list[index]+(1.0/(N*K))
    lp.obj[:] = coeff_list
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

    #print len(t)
    lp.matrix = t
    lp.simplex()
  #  print 'Z = %g;' % lp.obj.value,  # Retrieve and print obj func value
   # print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # Print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    #print labeling.T.shape[0],labeling.T.shape[1]
    ymax = (labeling.T,N,K)
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
    print 'number of 1s: %d' % c1
    print 'number of 0s: %d' % c0
    print 'number of 0.5s: %d' % ch
    print 'number of 0s: %d' % cr
    score = asarray(w_mat*x*ymax[0])[0][0];
    score2 = sm.svm_model.classify(psi(x,ymax,sm,sparm))
    print "objective value w/ const= ", (lp.obj.value+(1.0/K))
    print 'score : ' , round(score,2), ' score2: ',score2;
    print 'loss: ',loss(Y,ymax,sparm)
    print '\n'
    if(lp.obj.value  > 1.1):
      assert (round(lp.obj.value+(1.0/K),2) ==  round(score+loss(Y,ymax,sparm),2))
    return ymax

def lp_inference(X,sm,sparm):
    
    K = sm.num_classes
    w = sm.w
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

    x = (X[0])#.todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = asmatrix(array(w_list))
    #print w_list
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

    #print len(t)
    lp.matrix = t
    lp.simplex()
  #  print 'Z = %g;' % lp.obj.value,  # Retrieve and print obj func value
   # print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # Print struct variable names and primal val
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    #print labeling.T.shape[0],labeling.T.shape[1]
    ymax = (labeling.T,N,K)
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
    print 'number of 1s: %d' % c1
    print 'number of 0s: %d' % c0
    print 'number of 0.5s: %d' % ch
    print 'number of 0s: %d' % cr
    score = asarray(w_mat*x*ymax[0])[0][0];
    score2 = sm.svm_model.classify(psi(x,ymax,sm,sparm))
    print "objective value = ", round(lp.obj.value,2) 
    print '\n score : ' , round(score,2), ' score2: ',score2;
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
    #y = (mat(ones((1,x[0].shape[1]))),x[2],sm.num_classes)
    #l = lp_inference(x,y,sm,sparm)
    l = lp_inference(x,sm,sparm)
    return l

def find_most_violated_constraint(x, y, sm, sparm):
    """Returns the most violated constraint for example (x,y)."""
    # Similar, but include the loss.
    l = lp_training(x,y,sm,sparm)
     
    #print l.T
    return l

def psi(x, y, sm, sparm):
    """Returns the combined feature vector Psi(x,y)."""
    # Return the product of x and y
    #print x[0].shape[0]
    #print x[0].shape[1]
    #print y.shape[0]
    #print y.shape[1]
    return svmapi.Sparse((x[0]*y[0]))

def loss(Y, Ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    N = Y[1]
    K = Y[2]
    y= Y[0]
    
   # print N,K,y.shape[0],y.shape[1]
    ybar = Ybar[0] 
    yDiff=y- ybar;
    sum=0.0;
    size=N*K

    for index in xrange(0,size):
        if yDiff[index]>0:
            sum+=yDiff[index,0]
        else:
            sum-=yDiff[index,0]
            
    return sum/size;

def evaluation_class_pr(Y,Ybar,K,N,spram):
    y = Y[0]   
    ybar = Ybar[0]
    truecount = zeros((K,1))
    predcount = zeros((K,1))
    tpcount = zeros((K,1))
    confusionMatrix=zeros((K,K))
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
    for label in xrange(0,K):
        if(predcount[label,0] != 0):
            prec[label,0] = tpcount[label,0]/float(predcount[label,0])
        if(truecount[label,0] !=0):
            recall[label,0] = tpcount[label,0]/float(truecount[label,0])
    return (tpcount,truecount,predcount,confusionMatrix,zeroClasses,multipleClasses,confusionMatrixWMultiple)

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

    #print "similarity is", sim/N
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
    
    #print "similarity is", sim/N
    return 1-(sim/N);


def eval_prediction(exnum, (x, y), ypred, sm, sparm, teststats):
    """Accumulate statistics about a single training example.

    Allows accumulated statistics regarding how well the predicted
    label ypred for pattern x matches the true label y.  The first
    time this function is called teststats is None.  This function's
    return value will be passed along to the next call to
    eval_prediction.  After all test predictions are made, the last
    value returned will be passed along to print_testing_stats.

    On the first call, that is, when exnum==0, teststats==None.  The
    default behavior is that the function does nothing."""
    if exnum==0: teststats = []
    print 'on example',exnum,'predicted',ypred[0].T,'where correct is',y[0].T
    print 'loss is',evaluation_loss(y, ypred, sm.num_classes , x[2], sparm)
    #teststats.append(evaluation_loss(y, ypred, sm.num_classes , x[2], sparm))
    teststats.append(evaluation_class_pr(y, ypred, sm.num_classes , x[2], sparm))
    return teststats


def print_testing_stats(sample, sm, sparm, teststats):
    """Print statistics once classification has finished.

    This is called after all test predictions are made to allow the
    display of any summary statistics that have been accumulated in
    the teststats object through use of the eval_prediction function.

    The default behavior is that nothing is printed."""


    avgp = zeros((sm.num_classes,1))
    avgr = zeros((sm.num_classes,1))
    tpcount = zeros((sm.num_classes,1))
    truecount = zeros((sm.num_classes,1))
    predcount = zeros((sm.num_classes,1))
    aggConfusionMatrix=zeros((sm.num_classes,sm.num_classes))
    aggConfusionMatrixWMultiple=zeros((sm.num_classes,sm.num_classes))
    aggZeroPreds=zeros((sm.num_classes,1))
    aggMultiplePreds=zeros((sm.num_classes,1))
    for t in teststats:
        tpcount += t[0]
        truecount += t[1]
        predcount += t[2]
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
        #avgp[label,0] = avgp[label,0]/len(teststats)
        #avgr[label,0] = avgr[label,0]/len(teststats)
        print "label ",label+1, " prec: " , avgp[label,0], " recall: " ,avgr[label,0], " tp: ", tpcount[label,0], " tc: ", truecount[label,0], " pc: ", predcount[label,0]
        total_tc +=  truecount[label,0]
        total_pc += predcount[label,0]
        total_tp += tpcount[label,0]
    print "tp: ", total_tp, " pc: ", total_pc, "tc: ", total_tc
    #print "Error per Test example: ", teststats
    print "confusion matrix:"
    print aggConfusionMatrix;

    print "confusion matrix with multiple semantics:"
    print aggConfusionMatrixWMultiple;

    print "num Zeros:"
    print aggZeroPreds;

    print "num Multiples:"
    print aggMultiplePreds;
