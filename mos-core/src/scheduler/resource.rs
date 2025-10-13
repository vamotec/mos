//! Manages shared system resources for tasks.

use std::collections::HashMap;
use std::hash::Hash;
use tokio::sync::Mutex;
use std::sync::Arc;

/// Represents a unique, identifiable resource in the system.
/// The `Eq`, `PartialEq`, and `Hash` traits are required to use it as a key in a HashMap.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub enum Resource {
    /// Represents a sensor device, identified by its path.
    Sensor(String),
    /// Represents a GPIO pin, identified by its number.
    Gpio(u32),
}

/// Manages the locking and releasing of shared resources.
/// This ensures that only one task can access a resource at a time.
#[derive(Debug)]
pub struct ResourceManager {
    // The state of all registered resources. `Arc<Mutex<...>>` allows for safe concurrent access.
    // The `HashMap` maps a resource to the ID of the task that currently holds the lock.
    // If a resource is not in the map or its value is `None`, it is considered available.
    state: Arc<Mutex<HashMap<Resource, Option<u64>>>>,
}

impl ResourceManager {
    /// Creates a new, empty `ResourceManager`.
    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Registers a new resource with the manager, making it available for acquisition.
    /// By default, a newly registered resource is unlocked.
    pub async fn register_resource(&self, resource: Resource) {
        let mut state = self.state.lock().await;
        state.insert(resource, None);
    }

    /// Attempts to acquire a lock on a resource for a given task.
    ///
    /// This is a non-blocking check.
    ///
    /// # Returns
    /// * `Ok(true)` if the lock was successfully acquired.
    /// * `Ok(false)` if the resource is currently locked by another task.
    /// * `Err(String)` if the resource has not been registered.
    pub async fn acquire(&self, resource: &Resource, task_id: u64) -> Result<bool, String> {
        let mut state = self.state.lock().await;
        
        match state.get_mut(resource) {
            Some(lock_holder) => {
                if lock_holder.is_none() {
                    // Resource is available, acquire it.
                    *lock_holder = Some(task_id);
                    log::info!("Task {} acquired resource {:?}", task_id, resource);
                    Ok(true)
                } else {
                    // Resource is held by another task.
                    log::warn!("Task {} failed to acquire resource {:?}: already locked by {:?}", task_id, resource, lock_holder);
                    Ok(false)
                }
            }
            None => {
                // Resource is not registered.
                Err(format!("Attempted to acquire an unregistered resource: {:?}", resource))
            }
        }
    }

    /// Releases a lock on a resource that was held by a task.
    ///
    /// # Returns
    /// * `Ok(true)` if the resource was successfully released.
    /// * `Ok(false)` if the task attempting to release the lock is not the current lock holder.
    /// * `Err(String)` if the resource has not been registered.
    pub async fn release(&self, resource: &Resource, task_id: u64) -> Result<bool, String> {
        let mut state = self.state.lock().await;

        match state.get_mut(resource) {
            Some(lock_holder) => {
                if *lock_holder == Some(task_id) {
                    // Task is the rightful owner, release the lock.
                    *lock_holder = None;
                    log::info!("Task {} released resource {:?}", task_id, resource);
                    Ok(true)
                } else {
                    // Another task or no task holds the lock.
                    log::error!("Task {} failed to release resource {:?}: not the owner. Current owner: {:?}", task_id, resource, lock_holder);
                    Ok(false)
                }
            }
            None => {
                // Resource is not registered.
                Err(format!("Attempted to release an unregistered resource: {:?}", resource))
            }
        }
    }
}

// Default implementation for easy creation.
impl Default for ResourceManager {
    fn default() -> Self {
        Self::new()
    }
}
