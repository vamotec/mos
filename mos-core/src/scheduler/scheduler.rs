use std::cmp::Ordering;
use std::collections::BinaryHeap;
use tokio::sync::mpsc;

// TaskCommand is defined here as it's specific to the scheduler's internal command processing.
#[derive(Debug)]
pub enum TaskCommand {
    AddTask(crate::types::Task),
}

// The Task struct is an internal implementation detail of the Scheduler.
#[derive(Debug)]
pub struct Task {
    id: u64,
    priority: u32,
}

impl Task {
    pub fn id(&self) -> u64 {
        self.id
    }
}

// Implement comparison traits to use Task in a BinaryHeap (priority queue).
impl PartialEq for Task {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}
impl Eq for Task {}

impl PartialOrd for Task {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Higher priority values should be processed first.
impl Ord for Task {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority.cmp(&other.priority)
    }
}

// The Scheduler is generic over the command type.
pub struct Scheduler {
    tasks: BinaryHeap<Task>,
    cmd_rx: mpsc::Receiver<TaskCommand>,
}

impl Scheduler {
    pub fn new() -> (Self, mpsc::Sender<TaskCommand>) {
        let (cmd_tx, cmd_rx) = mpsc::channel(100);
        let scheduler = Self {
            tasks: BinaryHeap::new(),
            cmd_rx,
        };
        (scheduler, cmd_tx)
    }

    pub async fn run(mut self) {
        log::info!("Scheduler is running.");
        while let Some(cmd) = self.cmd_rx.recv().await {
            match cmd {
                TaskCommand::AddTask(task_dto) => {
                    log::info!("Adding new task: id={}, priority={}", task_dto.id, task_dto.priority);
                    let task = Task {
                        id: task_dto.id,
                        priority: task_dto.priority,
                    };
                    self.tasks.push(task);
                    // In a real implementation, we would pop from the heap and execute tasks.
                }
            }
        }
        log::info!("Scheduler is shutting down.");
    }
}
