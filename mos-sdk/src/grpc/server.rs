// mos-sdk/src/server.rs
use super::mos_sdk;
use super::MosSdk;
use tonic::{Request, Response, Status};

#[derive(Clone)]
pub struct MosSdkService {
    sdk: MosSdk,
}

#[tonic::async_trait]
impl mos_sdk::mos_sdk_server::MosSdk for MosSdkService {
    async fn move_(
        &self,
        request: Request<mos_sdk::MoveRequest>,
    ) -> Result<Response<mos_sdk::MoveResponse>, Status> {
        let joint_targets = request.into_inner().joint_targets;
        self.sdk
            .move_to(joint_targets)
            .await
            .map_err(|e| Status::internal(e.to_string()))?;
        Ok(Response::new(mos_sdk::MoveResponse {
            success: true,
            message: "Moved successfully".to_string(),
        }))
    }

    async fn get_image(
        &self,
        request: Request<mos_sdk::GetImageRequest>,
    ) -> Result<Response<mos_sdk::GetImageResponse>, Status> {
        let camera_id = request.into_inner().camera_id;
        let image_data = self
            .sdk
            .get_image(&camera_id)
            .await
            .map_err(|e| Status::internal(e.to_string()))?;
        Ok(Response::new(mos_sdk::GetImageResponse {
            image_data,
            timestamp: chrono::Utc::now().timestamp(),
        }))
    }

    async fn start_skill(
        &self,
        request: Request<mos_sdk::StartSkillRequest>,
    ) -> Result<Response<mos_sdk::StartSkillResponse>, Status> {
        let skill_id = request.into_inner().skill_id;
        self.sdk
            .start_skill(&skill_id)
            .await
            .map_err(|e| Status::internal(e.to_string()))?;
        Ok(Response::new(mos_sdk::StartSkillResponse {
            success: true,
            message: "Skill started".to_string(),
        }))
    }

    async fn stop_skill(
        &self,
        request: Request<mos_sdk::StopSkillRequest>,
    ) -> Result<Response<mos_sdk::StopSkillResponse>, Status> {
        let skill_id = request.into_inner().skill_id;
        self.sdk
            .stop_skill(&skill_id)
            .await
            .map_err(|e| Status::internal(e.to_string()))?;
        Ok(Response::new(mos_sdk::StopSkillResponse {
            success: true,
            message: "Skill stopped".to_string(),
        }))
    }

    async fn upload_metrics(
        &self,
        request: Request<mos_sdk::UploadMetricsRequest>,
    ) -> Result<Response<mos_sdk::UploadMetricsResponse>, Status> {
        let metrics = request.into_inner().metrics;
        self.sdk
            .upload_metrics(metrics)
            .await
            .map_err(|e| Status::internal(e.to_string()))?;
        Ok(Response::new(mos_sdk::UploadMetricsResponse {
            success: true,
            message: "Metrics uploaded".to_string(),
        }))
    }
}

impl MosSdkService {
    pub fn new(sdk: MosSdk) -> Self {
        Self { sdk }
    }
}