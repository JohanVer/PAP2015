#include <iostream>
#include <dlib/svm.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace dlib;

typedef matrix<double, 3, 1> sample_type;

template <class T>
std::vector<T> getSubset( std::vector<T> &in , size_t subset_size){
    std::vector<T> out;
    for(size_t i = 0; i < subset_size; i++){
        out.push_back(in.at(i));
    }
    return out;
}

int main()
{

    typedef radial_basis_kernel<sample_type> kernel_type;

    std::vector<sample_type> samples_full;
    std::vector<double> labels_full;

    // Now let's put some data into our samples and labels objects.  We do this by looping
    // over a bunch of points and labeling them according to their distance from the
    // origin.

    cv::Mat pic = cv::imread("/home/johan/Desktop/filternull_org.png",CV_LOAD_IMAGE_COLOR);
    cv::Mat pic_label = cv::imread("/home/johan/Desktop/filternull_label.png",CV_LOAD_IMAGE_COLOR);

    size_t positive_counter = 0;
    size_t negative_counter = 0;

    // Get positive examples

    for(size_t r = 0; r < pic.rows; r++){
        for(size_t c = 0; c < pic.cols; c++){
            cv::Vec3b pix = pic.at<cv::Vec3b>(r, c);
            cv::Vec3b pix_label = pic_label.at<cv::Vec3b>(r, c);
            sample_type new_sample;

            if(pix_label[1] == 0xff){
                new_sample(0) = pix[0];
                new_sample(1) = pix[1];
                new_sample(2) = pix[2];
                samples_full.push_back(new_sample);
                labels_full.push_back(+1);
                positive_counter++;
            }
        }
    }

    // Get negative examples

    for(size_t r = 0; r < pic.rows; r++){
        if(negative_counter >= positive_counter){
            break;
        }
        for(size_t c = 0; c < pic.cols; c++){
            cv::Vec3b pix = pic.at<cv::Vec3b>(r, c);
            cv::Vec3b pix_label = pic_label.at<cv::Vec3b>(r, c);
            sample_type new_sample;

            if(pix_label[2] == 0xff){
                new_sample(0) = pix[0];
                new_sample(1) = pix[1];
                new_sample(2) = pix[2];
                samples_full.push_back(new_sample);
                labels_full.push_back(-1);
                negative_counter++;
                if(negative_counter >= positive_counter){
                    break;
                }
            }
        }
    }

    std::cerr << "Size pos samples: " << positive_counter << " , negative: " << negative_counter << std::endl;


    vector_normalizer<sample_type> normalizer;
    // let the normalizer learn the mean and standard deviation of the samples
    normalizer.train(samples_full);
    // now normalize each sample
    for (unsigned long i = 0; i < samples_full.size(); ++i)
        samples_full[i] = normalizer(samples_full[i]);

    randomize_samples(samples_full, labels_full);

    std::vector<sample_type> samples;
    std::vector<double> labels;

    //samples = samples_full;
    //labels = labels_full;

    samples = getSubset(samples_full, 5000);
    labels = getSubset(labels_full, 5000);

    // The nu parameter has a maximum value that is dependent on the ratio of the +1 to -1
    // labels in the training data.  This function finds that value.
    const double max_nu = maximum_nu(labels);

    // here we make an instance of the svm_nu_trainer object that uses our kernel type.
    svm_nu_trainer<kernel_type> trainer;

    /*
    // Now we loop over some different nu and gamma values to see how good they are.  Note
    // that this is a very simple way to try out a few possible parameter choices.  You
    // should look at the model_selection_ex.cpp program for examples of more sophisticated
    // strategies for determining good parameter choices.
    cout << "doing cross validation" << endl;
    for (double gamma = 0.00001; gamma <= 1; gamma *= 5)
    {
        for (double nu = 0.00001; nu < max_nu; nu *= 5)
        {
            // tell the trainer the parameters we want to use
            trainer.set_kernel(kernel_type(gamma));
            trainer.set_nu(nu);

            cout << "gamma: " << gamma << "    nu: " << nu;
            // Print out the cross validation accuracy for 3-fold cross validation using
            // the current gamma and nu.  cross_validate_trainer() returns a row vector.
            // The first element of the vector is the fraction of +1 training examples
            // correctly classified and the second number is the fraction of -1 training
            // examples correctly classified.
            cout << "     cross validation accuracy: " << cross_validate_trainer(trainer, samples, labels, 3);
        }
    }
    */

    // From looking at the output of the above loop it turns out that a good value for nu
    // and gamma for this problem is 0.15625 for both.  So that is what we will use.

    // Now we train on the full set of data and obtain the resulting decision function.  We
    // use the value of 0.15625 for nu and gamma.  The decision function will return values
    // >= 0 for samples it predicts are in the +1 class and numbers < 0 for samples it
    // predicts to be in the -1 class.
    trainer.set_kernel(kernel_type(0.15625));
    trainer.set_nu(0.15625);
    typedef decision_function<kernel_type> dec_funct_type;
    typedef normalized_function<dec_funct_type> funct_type;


    funct_type learned_function;

    learned_function.normalizer = normalizer;  // save normalization information
    learned_function.function = trainer.train(samples, labels); // perform the actual SVM training and save the results

    cout << "\nnumber of support vectors in our learned_function is "
        << learned_function.function.basis_vectors.size() << endl;

    learned_function.function = reduced2(trainer,3).train(samples, labels);

    serialize("/home/johan/Desktop/svm_function.dat") << learned_function;


    // And similarly for the probabilistic_decision_function:

    // We can also train a decision function that reports a well conditioned probability
    // instead of just a number > 0 for the +1 class and < 0 for the -1 class.  An example
    // of doing that follows:

    typedef probabilistic_decision_function<kernel_type> probabilistic_funct_type;
    typedef normalized_function<probabilistic_funct_type> pfunct_type;

    pfunct_type learned_pfunct;
    learned_pfunct.normalizer = normalizer;
    learned_pfunct.function = train_probabilistic_decision_function(trainer, samples, labels, 3);

    // Now we have a function that returns the probability that a given sample is of the +1 class.

    // print out the number of support vectors in the resulting decision function.
    // (it should be the same as in the one above)
    cout << "\nnumber of support vectors in our learned_pfunct is "
         << learned_pfunct.function.decision_funct.basis_vectors.size() << endl;

    // Another thing that is worth knowing is that just about everything in dlib is
    // serializable.  So for example, you can save the learned_pfunct object to disk and
    // recall it later like so:
    serialize("/home/johan/Desktop/saved_function.dat") << learned_pfunct;

    // Now let's open that file back up and load the function object it contains.
    deserialize("/home/johan/Desktop/saved_function.dat") >> learned_pfunct;

    // Lastly, note that the decision functions we trained above involved well over 200
    // basis vectors.  Support vector machines in general tend to find decision functions
    // that involve a lot of basis vectors.  This is significant because the more basis
    // vectors in a decision function, the longer it takes to classify new examples.  So
    // dlib provides the ability to find an approximation to the normal output of a trainer
    // using fewer basis vectors.

    // Here we determine the cross validation accuracy when we approximate the output using
    // only 10 basis vectors.  To do this we use the reduced2() function.  It takes a
    // trainer object and the number of basis vectors to use and returns a new trainer
    // object that applies the necessary post processing during the creation of decision
    // function objects.
    cout << "\ncross validation accuracy with only 5 support vectors: "
         << cross_validate_trainer(reduced2(trainer,3), samples, labels, 3);

    // Let's print out the original cross validation score too for comparison.
    //cout << "cross validation accuracy with all the original support vectors: "
    //     << cross_validate_trainer(trainer, samples, labels, 3);

    // When you run this program you should see that, for this problem, you can reduce the
    // number of basis vectors down to 10 without hurting the cross validation accuracy.


    // To get the reduced decision function out we would just do this:
    learned_function.function = reduced2(trainer,3).train(samples, labels);
    // And similarly for the probabilistic_decision_function:
    learned_pfunct.function = train_probabilistic_decision_function(reduced2(trainer,3), samples, labels, 3);

    serialize("/home/johan/Desktop/prob_function.dat") << learned_pfunct;
}


