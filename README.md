# 步道坡线规划台

本地运行的步道坡线规划小工具：Python / FastAPI 负责计算，页面使用原生 HTML / CSS / JavaScript（无前端框架、无构建步骤）。

## 一条命令启动

```bash
docker compose up --build
```

启动后用浏览器访问：

- **http://localhost:8000**

服务端口：**8000**（宿主机与容器均为 8000，可在 `docker-compose.yml` 的 `ports` 中修改左侧宿主机端口，例如 `8080:8000`）。

停止服务：`Ctrl+C`，后台运行用 `docker compose up --build -d`，清理用 `docker compose down`。

## 功能与规则

- 可编辑 **3～16** 个测点，相邻测点相隔 **1 米**。
- 各点**地面标高**为 **0～200 的整数厘米**；方案标高取值范围相同。
- **首、尾方案标高必须等于对应地面标高**（页面中首尾输入框锁定）。
- 相邻方案点高差绝对值 **≤ 8 cm**。
- **任意连续 5 段中至少有 1 段水平段**（水平段 = 相邻方案点高差为 0）。
- 优化目标：各测点 `|方案标高 − 地面标高|` 之和（总调整量）最小；
  最小总调整量并列时，取**方案标高序列字典序最小**的方案。
- 页面叠画地面线（棕色虚线）与方案坡线（绿色实线），水平段以金色加粗标出；
  并列出每段高差/方向/是否水平段、逐点调整量、水平段数量与总调整量。
- 非法输入（测点数越界、非整数、标高超出 0～200 等）或无解（坡度/水平段限制无法同时满足）时，页面与接口均给出明确原因。

求解采用动态规划：以“当前标高 + 结尾连续非水平段数 (0～4)”为状态倒序递推，
再按标高从小到大恢复路径，天然得到总调整量最小且字典序最小的序列；16 个测点规模下耗时约几十毫秒。

## 接口

`POST /api/plan`

请求：

```json
{ "ground": [12, 14, 9, 6, 10, 16, 20, 18, 12, 8] }
```

响应（200，节选）：

```json
{
  "ok": true,
  "points": [
    { "index": 1, "ground": 12, "design": 12, "adjustment": 0 }
  ],
  "segments": [
    { "index": 1, "from": 12, "to": 14, "diff": 2, "horizontal": false }
  ],
  "totalAdjustment": 5,
  "horizontalCount": 2
}
```

- 非法输入返回 `400`，`error` 字段说明原因（如“第 2 个测点的地面标高 999 超出 0~200 厘米范围。”）。
- 无可行方案返回 `422`，例如起终点相对坡度过大：`reason: "unreachable_endpoint"`，并附可达范围等 `detail`。

## 不用 Docker 的本地运行方式

需要 Python 3.11+：

```bash
pip install -r requirements.txt
python app/main.py
# 或：uvicorn main:app --host 0.0.0.0 --port 8000 --app-dir app
```

访问 http://localhost:8000 。

## 目录结构

```
.
├── app
│   ├── main.py            # FastAPI 应用：输入校验 + 动态规划求解
│   └── static
│       └── index.html     # 单页前端（原生 HTML/CSS/JS + SVG 绘图）
├── Dockerfile
├── docker-compose.yml
├── .dockerignore
└── requirements.txt
```
