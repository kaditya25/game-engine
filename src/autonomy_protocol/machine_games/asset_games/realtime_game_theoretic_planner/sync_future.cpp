#include "sync_future.h"

void syncFuture::reserve(const int num_vecs, const int eig_vec_size)
{
  this->data_.resize(num_vecs);
  for( Eigen::VectorXd& vec : this->data_ )
  {
    vec.resize(eig_vec_size);
  }
}

void syncFuture::write_data(std::vector<Eigen::VectorXd>& data)
{
  if(this->failsafe_)
    std::cerr << "syncFuture: Write to data without resetting failsafe check, condition_variable will be bypassed\n";

  auto lk = this->get_write_lock();
  this->data_ = data;
  this->write_complete();
  this->notify_all();
}

    void syncFuture::release_write_lock(std::unique_lock<std::shared_timed_mutex>& lk)
{
  lk.unlock();
  this->write_complete();
  this->notify_all();
}

std::shared_lock<std::shared_timed_mutex> syncFuture::wait_read()
{
  auto lk = this->get_shared_lock();
  this->wait(lk);
  return lk; 
}

std::unique_lock<std::shared_timed_mutex> syncFuture::get_write_lock()
{
  std::unique_lock<std::shared_timed_mutex> lk(this->m_);
  return lk;
}

std::shared_lock<std::shared_timed_mutex> syncFuture::get_shared_lock()
{
  std::shared_lock<std::shared_timed_mutex> lk(this->m_);
  return lk;
}

void syncFuture::wait(std::shared_lock<std::shared_timed_mutex>& lk)
{
  this->cv_.wait(lk, 
      [this]{return this->failsafe_.load();});
}

std::string syncFuture::print()
{
  std::stringstream stream;
  if(!this->data_.size() || !this->data_[0].size())
  { std::string str;
    return str;
  }

  for (int vec_idx=0; vec_idx < this->data_.size(); vec_idx++)
  {
    stream << "--    --\t";
  }
  stream << "\n";
  for (int eig_idx=0; eig_idx < this->data_[0].size(); eig_idx++)
  {
    for (int vec_idx=0; vec_idx < this->data_.size(); vec_idx++)
    {
      stream << "| ";
      stream << std::fixed << std::setprecision(2) << (this->data_[vec_idx][eig_idx]); 
      stream << " |";
      if(vec_idx != this->data_.size()-1)
        stream << "\t";
    }
        stream << "\n";
  }
  for (int vec_idx=0; vec_idx < this->data_.size(); vec_idx++)
  {
    stream << "--    --\t";
  }
  return stream.str();
}

// pass through functions
void syncFuture::write_complete()
{ this->failsafe_ = true;}
void syncFuture::reset_failsafe()
{ this->failsafe_ = false;}
void syncFuture::notify_one()
{ this-> cv_.notify_one();}
void syncFuture::notify_all()
{ this-> cv_.notify_all();}
