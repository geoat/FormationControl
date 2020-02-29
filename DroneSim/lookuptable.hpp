#ifndef LOOKUPTABLE_HPP
#define LOOKUPTABLE_HPP

#include <exception>
#include <utility>
#include <memory>

//class for holding the value of k and f
template <typename data_type=float>
struct k_f_pair
{
    typedef data_type type;
    int k;
    data_type f;

    k_f_pair()= default;
    k_f_pair(const int& k, const data_type& f):k(k),f(f)
    {
        
    }
};

//class which helps in finding k and f for a given value with respect to a given vector
//k - index of element in the array which is just below the given element
//f - normalized position of the given element w.r.t the element at index k
template <typename array_type, typename value_type=array_type>
class Elements_Array
{
    public:
    typedef k_f_pair<value_type> k_f_pair_type;
    int array_length =0 ; 

    private:
    array_type* array = nullptr;
    

    public:
    Elements_Array(){}
    // constructor with a initializer list
    Elements_Array(std::initializer_list<array_type> list)
    {
        array_length = (int)list.size();
        array = new array_type[array_length];

        std::uninitialized_copy(list.begin(),list.end(), array);
    }

    //copy constructor
    Elements_Array(const Elements_Array<array_type>& other )
    {
        //std::cout<<"called_copyco"<<std::endl;
        if(array!=nullptr)
            delete[] array;
        array_length = other.array_length;
        array = new array_type[array_length];

        for(auto i = 0; i < array_length; i++)
        {
            array[i] = other.array[i];
        }
        
    }

    //copy assignment constructor
    Elements_Array<array_type> operator=(const Elements_Array<value_type>& other) 
    {
        //std::cout<<"called"<<std::endl;
        if(array!=nullptr)
            delete[] array;
        this->array_length = other.array_length;
        this->array = new array_type[this->array_length];

        for(auto i = 0; i < this->array_length; i++)
        {
            this->array[i] = other.array[i];
            //std::cout<<this->array[i]<<std::endl;
        }
        return *this;
    }


    //method to get k and f values
    auto get_k_f(const value_type& value) const ->decltype(k_f_pair<array_type>(0,value))
    {
        k_f_pair_type result;

              

        //find closest element with search  
        int i=0;
        for(; i < array_length; i++)
        {
            if(value<array[i])
            {
                break;
            }
        }
        result.k = i-1 + ((i==0)&1) - ((i==(array_length))&1) ; 
        result.f = ((typename k_f_pair_type::type)value-array[result.k])/(array[result.k+1]-array[result.k]);
        return result;
    }

    //function to fo interpolation on the elements
    value_type interpolate(const k_f_pair_type& kfpair) const
    {

        return (value_type)array[kfpair.k]+(value_type)(array[kfpair.k+1]-array[kfpair.k])*kfpair.f;  
    
    }

    // //functio to print elements 
    // void print() const
    // {
    //     std::cout<<"Array Address:"<<this->array<<std::endl;
    //     std::cout<<"Array:"<<std::endl;

    //     for(int i = 0; i < this->array_length; i++)
    //     {
    //         std::cout<<"Element["<<i<<"]:"<<this->array[i]<<std::endl;
    //     }
        
    // }


    ~Elements_Array()
    {

        if(array!=nullptr){

            delete[] array;

        }
        array = nullptr;
        
    }
};




//OneDLookUpTable with multple output values
template <typename data_type,typename value_type=data_type, int NumberOfOutputs=0>
class OneDLookUpTable
{
    typedef Elements_Array<data_type,value_type> Dtype;
    Dtype x;
    Dtype OutputValues[NumberOfOutputs];
    


    public:

    template<typename T, typename ...Ts>
    OneDLookUpTable(Elements_Array<T> head, Ts... list):x(head){
        if(NumberOfOutputs!=sizeof...(list)) throw "provide exact number of output value arrays";
        save_OutputValues({list...},x.array_length);
    }

    void save_OutputValues(std::initializer_list<Dtype> list, int requiredNumberofElements)
    {
        int i = 0;
        for( auto elem : list )
        {
            if(elem.array_length!=requiredNumberofElements) throw "output array length mismatches";
            OutputValues[i] = elem;
            
            ++i;
        }   
    }

    OneDLookUpTable()
    {
        
    }


    ~OneDLookUpTable()
    {
        
    }

    //copy assignment constructor
    OneDLookUpTable<data_type,value_type,NumberOfOutputs> operator=(const OneDLookUpTable<data_type,value_type,NumberOfOutputs>& other) 
    {
        x = other.x;
        for(auto i = 0; i < NumberOfOutputs; i++)
        {
            OutputValues[i] = other.OutputValues[i];
        }
        
        return *this;
    }


    //function to get interploated results in an array 
    void interpolate(value_type (&result)[NumberOfOutputs], const value_type& value) const
    {
        typename Dtype::k_f_pair_type kf_pair = x.get_k_f(value);
        // std::cout<<"k:"<<kf_pair.k<<std::endl;
        // std::cout<<"f:"<<kf_pair.f<<std::endl;
        for(auto i = 0; i < NumberOfOutputs; i++)
        {

            result[i]=OutputValues[i].interpolate(kf_pair);

        }
        
    }

    value_type interpolate(const value_type& value,const int& outIndex) const
    {
        typename Dtype::k_f_pair_type kf_pair = x.get_k_f(value);
        return OutputValues[outIndex].interpolate(kf_pair);
    }




};
//Two Dimesnional Lookup Table with single output
template <typename data_type,typename value_type=data_type>
class TwoDOutputValues
{
    int rowLength;
    int columnLength;
    data_type** array = nullptr;
    
    public:

    TwoDOutputValues()
    {

    }
    // constructor with a initializer list
    TwoDOutputValues(std::initializer_list<std::initializer_list<data_type>> listoflist)
    {
        rowLength = listoflist.size();
        columnLength = listoflist.begin()->size();
        //std::cout<<"mxn="<<rowLength<<"x"<<columnLength<<std::endl;
        array = new data_type*[rowLength];
        
        for(auto i = listoflist.begin(); i != listoflist.end(); i++)
        {
            if(i->size()!=columnLength) throw "column length missmatch";
            int pos = std::distance(listoflist.begin(),i);
            array[pos] = new data_type[columnLength];

            std::uninitialized_copy(i->begin(),i->end(), array[pos]);
        }
        
    }

    
    //copy constructor
    TwoDOutputValues(const TwoDOutputValues<data_type>& other )
    {
        clear_allocation();
        rowLength = other.rowLength;
        columnLength = other.columnLength;
        array = new data_type*[rowLength];

        for(int i = 0; i < rowLength; i++)
        {
            array[i]=new data_type[columnLength];
            std::uninitialized_copy(other.array[i],other.array[i]+columnLength, array[i]);
            /*for(int j = 0; j < columnLength; j++)
            {
                array[i][j]=other.array[i][j];
            }*/
            
        }
        
    }

    //copy assignment constructor
    TwoDOutputValues<data_type> operator=(const TwoDOutputValues<data_type>& other) 
    {
        clear_allocation();

        rowLength = other.rowLength;
        columnLength = other.columnLength;
        array = new data_type*[rowLength];

        for(int i = 0; i < rowLength; i++)
        {
            array[i]=new data_type[columnLength];
            std::uninitialized_copy(other.array[i],other.array[i]+columnLength, array[i]);
            /*for(int j = 0; j < columnLength; j++)
            {
                array[i][j]=other.array[i][j];

            }*/
        }
        return *this;
    }

    void clear_allocation()
    {
        if(array!=nullptr)
        {
            //std::cout<<"allcalled"<<std::endl;
            for(int i = 0; i < rowLength; i++)
            {
                if(array[i]!=nullptr)
                {
                    delete[] array[i];
                }
            }
            delete[] array;
        }
    }
    

    //Destructor
    ~TwoDOutputValues()
    {
        //std::cout<<"called";
        clear_allocation();
    }

    //interpolate function implmentation
    template <typename T> 
    value_type interpolate(const k_f_pair<T>& x,const k_f_pair<T>& y) const
    {
        //selecting the elements
        data_type a11 = array[x.k][y.k];
        data_type a12 = array[x.k][y.k+1];
        data_type a21 = array[x.k+1][y.k];
        data_type a22 = array[x.k+1][y.k+1];


        data_type ax1 = (value_type)a11+(value_type)(a21-a11)*x.f;
        data_type ax2 = (value_type)a12+(value_type)(a22-a12)*x.f;
        return ax1+(ax2-ax1)*y.f;

    }



};


//TwoDLookUpTable with signle output value
template <typename data_type,typename value_type=data_type>
class TwoDLookUpTable
{
    typedef Elements_Array<data_type,value_type> Dtype;
    Dtype x;
    Dtype y;
    TwoDOutputValues<data_type,value_type> OutputValues;

    public:
    TwoDLookUpTable(){}
    template<typename T, typename U>
    TwoDLookUpTable(Elements_Array<T> x,Elements_Array<T> y, TwoDOutputValues<U> OutputValues):x(x),y(y),OutputValues(OutputValues)
    {

    }

    template <typename T, typename U> 
    value_type interpolate(const T& xvalue,const U& yvalue) const
    {
            typename Dtype::k_f_pair_type kf_pair_x = x.get_k_f(xvalue);
            typename Dtype::k_f_pair_type kf_pair_y = y.get_k_f(yvalue);
            return OutputValues.interpolate(kf_pair_x,kf_pair_y);

    }
    ~TwoDLookUpTable(){}
};

#endif